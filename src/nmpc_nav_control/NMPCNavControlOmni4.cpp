#include "nmpc_nav_control/NMPCNavControlOmni4.h"

namespace nmpc_nav_control {

NMPCNavControlOmni4::NMPCNavControlOmni4(double dt, double l1_plus_l2) : NMPCNavControl(dt), 
                                                                         l1_plus_l2_(l1_plus_l2)
{
    // Initialize MPC
    mpc_capsule_ = omni4amr_acados_create_capsule();
    int create_status = omni4amr_acados_create(mpc_capsule_);
    processCreateStatus(create_status);

    for(unsigned int i = 0; i < OMNI4AMR_NX; i++) acados_in_.x0[i] = 0.0;
    for(unsigned int i = 0; i < OMNI4AMR_NU; i++) acados_out_.u0[i] = 0.0;
}

bool NMPCNavControlOmni4::run(const Pose& robot_pose, const std::list<Pose>& traj_ref, 
                              CmdVel& robot_vel_ref, double& cpu_time)
{
    // Set initial state
    acados_in_.x0[x] = robot_pose.x;
    acados_in_.x0[y] = robot_pose.y;
    acados_in_.x0[theta] = robot_pose.theta;
    
    ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims, 
                                  mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                  0, "lbx", acados_in_.x0);
    ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims, 
                                  mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                  0, "ubx", acados_in_.x0);
    
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
    int acados_status = omni4amr_acados_solve(mpc_capsule_);
    if (!processAcadosStatus(acados_status)) { return false; }

    acados_out_.status = acados_status;
    acados_out_.kkt_res = (double)mpc_capsule_->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule_->nlp_solver, "time_tot", &acados_out_.cpu_time);
    cpu_time = acados_out_.cpu_time*1000;

    ocp_nlp_out_get(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims, 
                    mpc_capsule_->nlp_out, 0, "u", (void*)acados_out_.u0);

    // Get reference velocities for the robot
    double new_v1_ref, new_v2_ref, new_v3_ref, new_v4_ref;
    new_v1_ref = acados_in_.x0[v1_ref] + acados_out_.u0[dv1_ref]*dt_;
    new_v2_ref = acados_in_.x0[v2_ref] + acados_out_.u0[dv2_ref]*dt_;
    new_v3_ref = acados_in_.x0[v3_ref] + acados_out_.u0[dv3_ref]*dt_;
    new_v4_ref = acados_in_.x0[v4_ref] + acados_out_.u0[dv4_ref]*dt_;

    // Cast CmdVel to CmdVelOmni4
    CmdVelOmni4* cmd_vel_omni4 = getCommandVelocity<CmdVelOmni4>(robot_vel_ref);
    if (!cmd_vel_omni4) { 
        throw std::runtime_error("Invalid command velocity type passed to run method.");
        return false;
    }
    inverseKinematrics(new_v1_ref, new_v2_ref, new_v3_ref, new_v4_ref,
                       cmd_vel_omni4->v, cmd_vel_omni4->vn, cmd_vel_omni4->w);

    // Setup next cycle initial state
    ocp_nlp_out_get(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims, 
                    mpc_capsule_->nlp_out, 1, "x", (void*)acados_in_.x0);

    return true;
}   


void NMPCNavControlOmni4::directKinematrics(const double v, const double vn, const double w, 
                                            double& v1, double& v2, double& v3, double& v4)
{
    v1 =  v - vn - 0.5 * l1_plus_l2_ * w;
    v2 = -v - vn - 0.5 * l1_plus_l2_ * w;
    v3 =  v + vn - 0.5 * l1_plus_l2_ * w;
    v4 = -v + vn - 0.5 * l1_plus_l2_ * w;
}

void NMPCNavControlOmni4::inverseKinematrics(const double v1, const double v2, const double v3, const double v4,
                                             double& v, double& vn, double& w)
{
    v  = ( v1 - v2 + v3 - v4) / 4.0;
    vn = (-v1 - v2 + v3 + v4) / 4.0;
    w  = (-v1 - v2 - v3 - v4) / (2.0 * l1_plus_l2_);
}

} // namespace nmpc_nav_control