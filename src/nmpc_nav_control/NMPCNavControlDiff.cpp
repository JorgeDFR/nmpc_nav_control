#include "nmpc_nav_control/NMPCNavControlDiff.h"
#include "nmpc_nav_control/utils.h"

namespace nmpc_nav_control {

NMPCNavControlDiff::NMPCNavControlDiff(double dt, double dist_left_to_right) : NMPCNavControl(dt),
                                                          dist_left_to_right_(dist_left_to_right)
{
    // Initialize MPC
    mpc_capsule_ = diff2amr_acados_create_capsule();
    int create_status = diff2amr_acados_create(mpc_capsule_);
    processCreateStatus(create_status);

    for(unsigned int i = 0; i < DIFF2AMR_NX; i++) acados_in_.x0[i] = 0.0;
    for(unsigned int i = 0; i < DIFF2AMR_NU; i++) acados_out_.u0[i] = 0.0;

    first_cycle = true;
    x_last = y_last = theta_last = 0.0;
}

bool NMPCNavControlDiff::run(const Pose& robot_pose, const Vel& robot_vel,
                             const std::list<Pose>& traj_ref,
                             CmdVel& robot_vel_ref, double& cpu_time)
{
    // Set initial state
    acados_in_.x0[x] = robot_pose.x;
    acados_in_.x0[y] = robot_pose.y;
    acados_in_.x0[theta] = robot_pose.theta;

    double vl_est, vr_est;
    directKinematrics(robot_vel.v, robot_vel.w, vl_est, vr_est);
    acados_in_.x0[vl] = vl_est;
    acados_in_.x0[vr] = vr_est;

    // ################################### TEMP (Debug) ###################################
    // double v, w;
    // if (first_cycle) {
    //     v = 0.0;
    //     w = 0.0;
    //     first_cycle = false;
    // } else {
    //     double vx = (robot_pose.x - x_last) / dt_;
    //     double vy = (robot_pose.y - y_last) / dt_;
    //     v = vx * cos(robot_pose.theta) + vy * sin(robot_pose.theta);
    //     w = normAngRad(theta_last - robot_pose.theta) / dt_;
    // }
    // x_last = robot_pose.x;
    // y_last = robot_pose.y;
    // theta_last = robot_pose.theta;

    // double vl_est, vr_est;
    // directKinematrics(v, w, vl_est, vr_est);
    // std::cout << "vl: "    << acados_in_.x0[vl] << " | vl_est: " << vl_est << std::endl
    //           << " | vr: " << acados_in_.x0[vr] << " | vr_est: " << vr_est << std::endl;
    // ####################################################################################

    ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                  mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                  0, "lbx", acados_in_.x0);
    ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                  mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                  0, "ubx", acados_in_.x0);

    // Unwrap reference angles
    double previous_theta = robot_pose.theta;
    auto it = traj_ref.begin();
    for (unsigned int i = 0; i <= DIFF2AMR_N; i++) {
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
    for (unsigned int i = 0; i <= DIFF2AMR_N; i++) {
        ocp_nlp_cost_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                               mpc_capsule_->nlp_in, i, "yref", acados_in_.yref[i]);
    }

    // Solve optimization problem
    int acados_status = diff2amr_acados_solve(mpc_capsule_);
    if (!processAcadosStatus(acados_status)) { return false; }

    acados_out_.status = acados_status;
    acados_out_.kkt_res = (double)mpc_capsule_->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule_->nlp_solver, "time_tot", &acados_out_.cpu_time);
    cpu_time = acados_out_.cpu_time*1000;

    ocp_nlp_out_get(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                    mpc_capsule_->nlp_out, 0, "u", (void*)acados_out_.u0);

    // Get reference velocities for the robot
    double new_vl_ref, new_vr_ref;
    new_vl_ref = acados_in_.x0[vl_ref] + acados_out_.u0[dvl_ref]*dt_;
    new_vr_ref = acados_in_.x0[vr_ref] + acados_out_.u0[dvr_ref]*dt_;

    // Cast CmdVel to CmdVelDiff
    CmdVelDiff* cmd_vel_diff = getCommandVelocity<CmdVelDiff>(robot_vel_ref);
    if (!cmd_vel_diff) {
        throw std::runtime_error("Invalid command velocity type passed to run method.");
        return false;
    }
    inverseKinematrics(new_vl_ref, new_vr_ref, cmd_vel_diff->v, cmd_vel_diff->w);

    // Setup next cycle initial state
    ocp_nlp_out_get(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                    mpc_capsule_->nlp_out, 1, "x", (void*)acados_in_.x0);

    acados_in_.x0[vl_ref] = new_vl_ref;
    acados_in_.x0[vr_ref] = new_vr_ref;

    return true;
}


void NMPCNavControlDiff::directKinematrics(const double v, const double w, double& vl, double& vr)
{
    vl =  v - 0.5 * dist_left_to_right_ * w;
    vr =  v + 0.5 * dist_left_to_right_ * w;
}

void NMPCNavControlDiff::inverseKinematrics(const double vl, const double vr, double& v, double& w)
{
    v  = ( vr + vl ) / 2.0;
    w  = ( vr - vl ) / dist_left_to_right_;
}

} // namespace nmpc_nav_control