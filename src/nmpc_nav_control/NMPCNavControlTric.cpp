#include "nmpc_nav_control/NMPCNavControlTric.h"

namespace nmpc_nav_control {

NMPCNavControlTric::NMPCNavControlTric(double dt, double dist_d, double tau_v, double tau_a, double v_max, double a_max,
                                       double alpha_min, double alpha_max, double dalpha_max) : NMPCNavControl(dt)
{
    // Initialize MPC
    mpc_capsule_ = tric3amr_acados_create_capsule();
    int create_status = tric3amr_acados_create(mpc_capsule_);
    processCreateStatus(create_status);

    robot_steering_wheel_angle_ = 0.0;

    for(unsigned int i = 0; i < TRIC3AMR_NX; i++) { acados_in_.x0[i] = 0.0; }
    for(unsigned int i = 0; i < TRIC3AMR_NU; i++) { acados_out_.u0[i] = 0.0; }

    acados_p_.p[SystemParametersMap::dist_d] = dist_d;
    acados_p_.p[SystemParametersMap::tau_v] = tau_v;
    acados_p_.p[SystemParametersMap::tau_a] = tau_a;
    acados_p_.x_min[0] = -v_max;
    acados_p_.x_max[0] =  v_max;
    acados_p_.x_min[1] = alpha_min;
    acados_p_.x_max[1] = alpha_max;
    acados_p_.u_min[0] = -a_max;
    acados_p_.u_max[0] =  a_max;
    acados_p_.u_min[1] = -dalpha_max;
    acados_p_.u_max[1] =  dalpha_max;

    for (unsigned int i = 0; i < TRIC3AMR_NY*TRIC3AMR_NY; i++) { acados_p_.W[i] = 0.0; }
    // acados_p_.W[0+(TRIC3AMR_NY) * 0] = 10.0;
    // acados_p_.W[1+(TRIC3AMR_NY) * 1] = 10.0;
    // acados_p_.W[2+(TRIC3AMR_NY) * 2] = 5.0;
    // acados_p_.W[3+(TRIC3AMR_NY) * 3] = 0.0;
    // acados_p_.W[4+(TRIC3AMR_NY) * 4] = 0.0;
    // acados_p_.W[5+(TRIC3AMR_NY) * 5] = 0.0;
    // acados_p_.W[6+(TRIC3AMR_NY) * 6] = 0.0;
    // acados_p_.W[7+(TRIC3AMR_NY) * 7] = 1.0;
    // acados_p_.W[8+(TRIC3AMR_NY) * 8] = 1.0;
    for (unsigned int i = 0; i < TRIC3AMR_NYN*TRIC3AMR_NYN; i++) { acados_p_.W_e[i] = 0.0; }
    // acados_p_.W_e[0+(TRIC3AMR_NYN) * 0] = 1000.0;
    // acados_p_.W_e[1+(TRIC3AMR_NYN) * 1] = 1000.0;
    // acados_p_.W_e[2+(TRIC3AMR_NYN) * 2] = 500.0;
    // acados_p_.W_e[3+(TRIC3AMR_NYN) * 3] = 0.0;
    // acados_p_.W_e[4+(TRIC3AMR_NYN) * 4] = 0.0;
    // acados_p_.W_e[5+(TRIC3AMR_NYN) * 5] = 0.0;
    // acados_p_.W_e[6+(TRIC3AMR_NYN) * 6] = 0.0;

    // Set model parameters
    for (unsigned int i = 0; i < TRIC3AMR_N; i++) {
        tric3amr_acados_update_params(mpc_capsule_, i, acados_p_.p, TRIC3AMR_NP);
    }

    // Set constraints bounds
    for (unsigned int i = 1; i <= TRIC3AMR_N; i++) {
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "lbx", acados_p_.x_min);
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "ubx", acados_p_.x_max);
    }

    for (unsigned int i = 0; i < TRIC3AMR_N; i++) {
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "lbu", acados_p_.u_min);
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "ubu", acados_p_.u_max);
    }

    // Set cost function weights
    // for (unsigned int i = 0; i < TRIC3AMR_N; i++) {
    //     ocp_nlp_cost_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
    //                            mpc_capsule_->nlp_in, i, "W", acados_p_.W);
    // }
    // ocp_nlp_cost_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
    //                        mpc_capsule_->nlp_in, TRIC3AMR_N, "W", acados_p_.W_e);
}

NMPCNavControlTric::~NMPCNavControlTric()
{
    tric3amr_acados_free(mpc_capsule_);
    tric3amr_acados_free_capsule(mpc_capsule_);
}

bool NMPCNavControlTric::run(const Pose& robot_pose,  const Vel& robot_vel,
                             const std::list<Pose>& traj_ref,
                             CmdVel& robot_vel_ref, double& cpu_time)
{
    // Set initial state
    acados_in_.x0[x] = robot_pose.x;
    acados_in_.x0[y] = robot_pose.y;
    acados_in_.x0[theta] = robot_pose.theta;

    acados_in_.x0[v] = robot_vel.v;
    acados_in_.x0[alpha] = robot_steering_wheel_angle_;

    ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                  mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                  0, "lbx", acados_in_.x0);
    ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                  mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                  0, "ubx", acados_in_.x0);

    // Unwrap reference angles
    double previous_theta = robot_pose.theta;
    auto it = traj_ref.begin();
    for (unsigned int i = 0; i <= TRIC3AMR_N; i++) {
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
    for (unsigned int i = 0; i <= TRIC3AMR_N; i++) {
        ocp_nlp_cost_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                               mpc_capsule_->nlp_in, i, "yref", acados_in_.yref[i]);
    }

    // Solve optimization problem
    int acados_status = tric3amr_acados_solve(mpc_capsule_);
    if (!processAcadosStatus(acados_status)) { return false; }

    acados_out_.status = acados_status;
    acados_out_.kkt_res = (double)mpc_capsule_->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule_->nlp_solver, "time_tot", &acados_out_.cpu_time);
    cpu_time = acados_out_.cpu_time*1000;

    ocp_nlp_out_get(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                    mpc_capsule_->nlp_out, 0, "u", (void*)acados_out_.u0);

    // Get reference velocities for the robot
    double new_v_ref, new_alpha_ref;
    new_v_ref = acados_in_.x0[v_ref] + acados_out_.u0[v_ref]*dt_;
    new_alpha_ref = acados_in_.x0[alpha_ref] + acados_out_.u0[alpha_ref]*dt_;

    // Cast CmdVel to CmdVelTric
    CmdVelTric* cmd_vel_tric = getCommandVelocity<CmdVelTric>(robot_vel_ref);
    if (!cmd_vel_tric) {
        throw std::runtime_error("Invalid command velocity type passed to run method.");
        return false;
    }
    cmd_vel_tric->v = new_v_ref;
    cmd_vel_tric->alpha = new_alpha_ref;

    // Setup next cycle initial state
    ocp_nlp_out_get(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                    mpc_capsule_->nlp_out, 1, "x", (void*)acados_in_.x0);


    acados_in_.x0[v_ref] = new_v_ref;
    acados_in_.x0[alpha_ref] = new_alpha_ref;

    return true;
}

bool NMPCNavControlTric::reset_mpc()
{
    tric3amr_acados_reset(mpc_capsule_, 1);
    return true;
}

} // namespace nmpc_nav_control