#include "nmpc_nav_control/NMPCNavControlDiff.h"
#include "nmpc_nav_control/utils.h"

namespace nmpc_nav_control {

NMPCNavControlDiff::NMPCNavControlDiff(double dt, double dist_b, double tau_v, double v_max, double a_max) : NMPCNavControl(dt)
{
    // Initialize MPC
    mpc_capsule_ = diff2amr_acados_create_capsule();
    int create_status = diff2amr_acados_create(mpc_capsule_);
    processCreateStatus(create_status);

    for (unsigned int i = 0; i < DIFF2AMR_NX; i++) { acados_in_.x0[i] = 0.0; }
    for (unsigned int i = 0; i < DIFF2AMR_NU; i++) { acados_out_.u0[i] = 0.0; }

    acados_p_.p[SystemParametersMap::dist_b] = dist_b;
    acados_p_.p[SystemParametersMap::tau_v] = tau_v;
    acados_p_.x_min[0] = acados_p_.x_min[1] = -v_max;
    acados_p_.x_max[0] = acados_p_.x_max[1] =  v_max;
    acados_p_.u_min[0] = acados_p_.u_min[1] = -a_max;
    acados_p_.u_max[0] = acados_p_.u_max[1] =  a_max;

    for (unsigned int i = 0; i < DIFF2AMR_NY*DIFF2AMR_NY; i++) { acados_p_.W[i] = 0.0; }
    // acados_p_.W[0+(DIFF2AMR_NY) * 0] = 10.0;
    // acados_p_.W[1+(DIFF2AMR_NY) * 1] = 10.0;
    // acados_p_.W[2+(DIFF2AMR_NY) * 2] = 5.0;
    // acados_p_.W[3+(DIFF2AMR_NY) * 3] = 0.0;
    // acados_p_.W[4+(DIFF2AMR_NY) * 4] = 0.0;
    // acados_p_.W[5+(DIFF2AMR_NY) * 5] = 0.0;
    // acados_p_.W[6+(DIFF2AMR_NY) * 6] = 0.0;
    // acados_p_.W[7+(DIFF2AMR_NY) * 7] = 1.0;
    // acados_p_.W[8+(DIFF2AMR_NY) * 8] = 1.0;
    for (unsigned int i = 0; i < DIFF2AMR_NYN*DIFF2AMR_NYN; i++) { acados_p_.W_e[i] = 0.0; }
    // acados_p_.W_e[0+(DIFF2AMR_NYN) * 0] = 1000.0;
    // acados_p_.W_e[1+(DIFF2AMR_NYN) * 1] = 1000.0;
    // acados_p_.W_e[2+(DIFF2AMR_NYN) * 2] = 500.0;
    // acados_p_.W_e[3+(DIFF2AMR_NYN) * 3] = 0.0;
    // acados_p_.W_e[4+(DIFF2AMR_NYN) * 4] = 0.0;
    // acados_p_.W_e[5+(DIFF2AMR_NYN) * 5] = 0.0;
    // acados_p_.W_e[6+(DIFF2AMR_NYN) * 6] = 0.0;

    // Set model parameters
    for (unsigned int i = 0; i < DIFF2AMR_N; i++) {
        diff2amr_acados_update_params(mpc_capsule_, i, acados_p_.p, DIFF2AMR_NP);
    }

    // Set constraints bounds
    for (unsigned int i = 1; i <= DIFF2AMR_N; i++) {
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "lbx", acados_p_.x_min);
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "ubx", acados_p_.x_max);
    }

    for (unsigned int i = 0; i < DIFF2AMR_N; i++) {
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "lbu", acados_p_.u_min);
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "ubu", acados_p_.u_max);
    }

    // Set cost function weights
    // for (unsigned int i = 0; i < DIFF2AMR_N; i++) {
    //     ocp_nlp_cost_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
    //                            mpc_capsule_->nlp_in, i, "W", acados_p_.W);
    // }
    // ocp_nlp_cost_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
    //                        mpc_capsule_->nlp_in, DIFF2AMR_N, "W", acados_p_.W_e);
}

NMPCNavControlDiff::~NMPCNavControlDiff()
{
    diff2amr_acados_free(mpc_capsule_);
    diff2amr_acados_free_capsule(mpc_capsule_);
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
    vl =  v - 0.5 * acados_p_.p[dist_b] * w;
    vr =  v + 0.5 * acados_p_.p[dist_b] * w;
}

void NMPCNavControlDiff::inverseKinematrics(const double vl, const double vr, double& v, double& w)
{
    v  = (vr + vl) / 2.0;
    w  = (vr - vl) / acados_p_.p[dist_b];
}

} // namespace nmpc_nav_control