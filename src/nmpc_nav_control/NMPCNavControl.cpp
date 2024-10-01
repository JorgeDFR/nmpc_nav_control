#include "nmpc_nav_control/NMPCNavControl.h"
#include "nmpc_nav_control/utils.h"

namespace nmpc_nav_control {

NMPCNavControl::NMPCNavControl(double l1_plus_l2, double dt) : l1_plus_l2_(l1_plus_l2), dt_(dt)
{
    // Initialize MPC
    mpc_capsule_ = omni4amr_acados_create_capsule();
    int create_status = omni4amr_acados_create(mpc_capsule_);
    if (create_status != 0) {
        std::stringstream error;
        error << "acados_create() returned status " << create_status << ".";
        throw std::runtime_error(error.str());
    }

    for(unsigned int i = 0; i < OMNI4AMR_NX; i++) acados_in_.x0[i] = 0.0;
    for(unsigned int i = 0; i < OMNI4AMR_NU; i++) acados_out_.u0[i] = 0.0;
}

bool NMPCNavControl::run(const Pose& robot_pose, std::list<Pose>& traj_ref, CmdVel& robot_vel_ref, double& cpu_time)
{   
    // Check end of trajectory
    double d = dist(robot_pose.x, robot_pose.y, traj_ref.begin()->x, traj_ref.begin()->y);
    double ang = normAngRad(robot_pose.theta - traj_ref.begin()->theta);
    if ((d <= 0.005) && (ang <= M_PI / 180.0)) {
        traj_ref.pop_front();
        robot_vel_ref.v = 0.0;
        robot_vel_ref.vn = 0.0;
        robot_vel_ref.w = 0.0;
        return true;
    }

    // Set initial state
    acados_in_.x0[x] = robot_pose.x;
    acados_in_.x0[y] = robot_pose.y;
    acados_in_.x0[theta] = robot_pose.theta;
    
    ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims, 
                                  mpc_capsule_->nlp_in, 0, "lbx", acados_in_.x0);
    ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims, 
                                  mpc_capsule_->nlp_in, 0, "ubx", acados_in_.x0);
    
    // Unwrap reference angles
    double previous_theta = robot_pose.theta;
    auto it = traj_ref.begin();
    for (unsigned int i = 0; i <= OMNI4AMR_N; i++) {
        if (it != traj_ref.end()) {
            acados_in_.yref[i][x] = it->x;
            acados_in_.yref[i][y] = it->y;
            acados_in_.yref[i][theta] = unwrapAngle(it->theta, previous_theta);
            previous_theta = acados_in_.yref[i][theta];
            it++;
        } else {
            acados_in_.yref[i][x] = acados_in_.yref[i-1][x];
            acados_in_.yref[i][y] = acados_in_.yref[i-1][y];
            acados_in_.yref[i][theta] = acados_in_.yref[i-1][theta];
        }
    }

    // Set reference trajectory
    for (unsigned int i = 0; i <= OMNI4AMR_N; i++) {            
        ocp_nlp_cost_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims, 
                               mpc_capsule_->nlp_in, i, "yref", acados_in_.yref[i]);
    }

    // Solve optimization problem
    acados_status_ = omni4amr_acados_solve(mpc_capsule_);
    if (acados_status_ != 0) {
        std::stringstream error;
        error << "acados_solve() returned status " << acados_status_ << ".";
        throw std::runtime_error(error.str());
        return false;
    }

    acados_out_.status = acados_status_;
    acados_out_.kkt_res = (double)mpc_capsule_->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule_->nlp_config, mpc_capsule_->nlp_solver, "time_tot", &acados_out_.cpu_time);
    cpu_time = acados_out_.cpu_time*1000;

    ocp_nlp_out_get(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims, 
                    mpc_capsule_->nlp_out, 0, "u", (void*)acados_out_.u0);

    // Get reference velocities for the robot
    double new_v1_ref, new_v2_ref, new_v3_ref, new_v4_ref;
    new_v1_ref = acados_in_.x0[v1_ref] + acados_out_.u0[dv1_ref]*dt_;
    new_v2_ref = acados_in_.x0[v2_ref] + acados_out_.u0[dv2_ref]*dt_;
    new_v3_ref = acados_in_.x0[v3_ref] + acados_out_.u0[dv3_ref]*dt_;
    new_v4_ref = acados_in_.x0[v4_ref] + acados_out_.u0[dv4_ref]*dt_;

    inverseKinematrics(new_v1_ref, new_v2_ref, new_v3_ref, new_v4_ref,
                       robot_vel_ref.v, robot_vel_ref.vn, robot_vel_ref.w);

    // Setup next cycle initial state
    ocp_nlp_out_get(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims, 
                    mpc_capsule_->nlp_out, 1, "x", (void*)acados_in_.x0);

    return true;
}   


void NMPCNavControl::directKinematrics(const double v, const double vn, const double w, 
                                       double& v1, double& v2, double& v3, double& v4)
{
    v1 =  v - vn - 0.5 * l1_plus_l2_ * w;
    v2 = -v - vn - 0.5 * l1_plus_l2_ * w;
    v3 =  v + vn - 0.5 * l1_plus_l2_ * w;
    v4 = -v + vn - 0.5 * l1_plus_l2_ * w;
}

void NMPCNavControl::inverseKinematrics(const double v1, const double v2, const double v3, const double v4,
                                        double& v, double& vn, double& w)
{
    v  = ( v1 - v2 + v3 - v4) / 4.0;
    vn = (-v1 - v2 + v3 + v4) / 4.0;
    w  = (-v1 - v2 - v3 - v4) / (2.0 * l1_plus_l2_);
}

double NMPCNavControl::unwrapAngle(double current, double previous) 
{
    double delta = current - previous;
    if (delta > M_PI) { current -= 2 * M_PI; } 
    else if (delta < -M_PI) { current += 2 * M_PI; }
    return current;
};

} // namespace nmpc_nav_control