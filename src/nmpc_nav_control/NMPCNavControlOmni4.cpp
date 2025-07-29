#include "nmpc_nav_control/NMPCNavControlOmni4.h"

namespace nmpc_nav_control {

NMPCNavControlOmni4::NMPCNavControlOmni4(double dt, double l1_plus_l2, double tau_v, double v_max, double a_max,
                                         std::vector<double> W_diag) : NMPCNavControl(dt)
{
    // Initialize MPC
    mpc_capsule_ = omni4amr_acados_create_capsule();
    int create_status = omni4amr_acados_create(mpc_capsule_);
    processCreateStatus(create_status);

    for (unsigned int i = 0; i < OMNI4AMR_NX; i++) { acados_in_.x0[i] = 0.0; }
    for (unsigned int i = 0; i < OMNI4AMR_NU; i++) { acados_out_.u0[i] = 0.0; }

    acados_p_.p[SystemParametersMap::l1_plus_l2] = l1_plus_l2;
    acados_p_.p[SystemParametersMap::tau_v] = tau_v;
    acados_p_.x_min[0] = acados_p_.x_min[1] = acados_p_.x_min[2] = acados_p_.x_min[3] = -v_max;
    acados_p_.x_max[0] = acados_p_.x_max[1] = acados_p_.x_max[2] = acados_p_.x_max[3] =  v_max;
    acados_p_.u_min[0] = acados_p_.u_min[1] = acados_p_.u_min[2] = acados_p_.u_min[3] = -a_max;
    acados_p_.u_max[0] = acados_p_.u_max[1] = acados_p_.u_max[2] = acados_p_.u_max[3] =  a_max;

    for (unsigned int i = 0; i < OMNI4AMR_NY*OMNI4AMR_NY; i++) { acados_p_.W[i] = 0.0; }
    acados_p_.W[0+(OMNI4AMR_NY) * 0] = W_diag[0];
    acados_p_.W[1+(OMNI4AMR_NY) * 1] = W_diag[1];
    acados_p_.W[2+(OMNI4AMR_NY) * 2] = W_diag[2];
    acados_p_.W[3+(OMNI4AMR_NY) * 3] = W_diag[3];
    acados_p_.W[4+(OMNI4AMR_NY) * 4] = W_diag[4];
    acados_p_.W[5+(OMNI4AMR_NY) * 5] = W_diag[5];
    acados_p_.W[6+(OMNI4AMR_NY) * 6] = W_diag[6];
    acados_p_.W[7+(OMNI4AMR_NY) * 7] = W_diag[7];
    acados_p_.W[8+(OMNI4AMR_NY) * 8] = W_diag[8];
    acados_p_.W[9+(OMNI4AMR_NY) * 9] = W_diag[9];
    acados_p_.W[10+(OMNI4AMR_NY) * 10] = W_diag[10];
    acados_p_.W[11+(OMNI4AMR_NY) * 11] = W_diag[11];
    acados_p_.W[12+(OMNI4AMR_NY) * 12] = W_diag[12];
    acados_p_.W[13+(OMNI4AMR_NY) * 13] = W_diag[13];
    acados_p_.W[14+(OMNI4AMR_NY) * 14] = W_diag[14];
    for (unsigned int i = 0; i < OMNI4AMR_NYN*OMNI4AMR_NYN; i++) { acados_p_.W_e[i] = 0.0; }
    acados_p_.W_e[0+(OMNI4AMR_NYN) * 0] = W_diag[0];
    acados_p_.W_e[1+(OMNI4AMR_NYN) * 1] = W_diag[1];
    acados_p_.W_e[2+(OMNI4AMR_NYN) * 2] = W_diag[2];
    acados_p_.W_e[3+(OMNI4AMR_NYN) * 3] = W_diag[3];
    acados_p_.W_e[4+(OMNI4AMR_NYN) * 4] = W_diag[4];
    acados_p_.W_e[5+(OMNI4AMR_NYN) * 5] = W_diag[5];
    acados_p_.W_e[6+(OMNI4AMR_NYN) * 6] = W_diag[6];
    acados_p_.W_e[7+(OMNI4AMR_NYN) * 7] = W_diag[7];
    acados_p_.W_e[8+(OMNI4AMR_NYN) * 8] = W_diag[8];
    acados_p_.W_e[9+(OMNI4AMR_NYN) * 9] = W_diag[9];
    acados_p_.W_e[10+(OMNI4AMR_NYN) * 10] = W_diag[10];

    // Set model parameters
    for (unsigned int i = 0; i < OMNI4AMR_N; i++) {
        omni4amr_acados_update_params(mpc_capsule_, i, acados_p_.p, OMNI4AMR_NP);
    }

    // Set constraints bounds
    for (unsigned int i = 1; i <= OMNI4AMR_N; i++) {
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "lbx", acados_p_.x_min);
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "ubx", acados_p_.x_max);
    }

    for (unsigned int i = 0; i < OMNI4AMR_N; i++) {
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "lbu", acados_p_.u_min);
        ocp_nlp_constraints_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                                      mpc_capsule_->nlp_in, mpc_capsule_->nlp_out,
                                      i, "ubu", acados_p_.u_max);
    }

    // Set cost function weights
    for (unsigned int i = 0; i < OMNI4AMR_N; i++) {
        ocp_nlp_cost_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                               mpc_capsule_->nlp_in, i, "W", acados_p_.W);
    }
    ocp_nlp_cost_model_set(mpc_capsule_->nlp_config, mpc_capsule_->nlp_dims,
                           mpc_capsule_->nlp_in, OMNI4AMR_N, "W", acados_p_.W_e);
}

NMPCNavControlOmni4::~NMPCNavControlOmni4()
{
    omni4amr_acados_free(mpc_capsule_);
    omni4amr_acados_free_capsule(mpc_capsule_);
}

bool NMPCNavControlOmni4::run(const Pose& robot_pose, const Vel& robot_vel,
                              const std::list<Pose>& traj_ref,
                              CmdVel& robot_vel_ref, double& cpu_time)
{
    // Set initial state
    acados_in_.x0[x] = robot_pose.x;
    acados_in_.x0[y] = robot_pose.y;
    acados_in_.x0[theta] = robot_pose.theta;

    double v1_est, v2_est, v3_est, v4_est;
    directKinematrics(robot_vel.v, robot_vel.vn, robot_vel.w,
                      v1_est, v2_est, v3_est, v4_est);
    acados_in_.x0[v1] = v1_est;
    acados_in_.x0[v2] = v2_est;
    acados_in_.x0[v3] = v3_est;
    acados_in_.x0[v4] = v4_est;

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

    acados_in_.x0[v1_ref] = new_v1_ref;
    acados_in_.x0[v2_ref] = new_v2_ref;
    acados_in_.x0[v3_ref] = new_v3_ref;
    acados_in_.x0[v4_ref] = new_v4_ref;

    return true;
}

bool NMPCNavControlOmni4::reset_mpc()
{
    omni4amr_acados_reset(mpc_capsule_, 1);
    return true;
}

void NMPCNavControlOmni4::directKinematrics(const double v, const double vn, const double w,
                                            double& v1, double& v2, double& v3, double& v4)
{
    v1 =  v - vn - 0.5 * acados_p_.p[l1_plus_l2] * w;
    v2 = -v - vn - 0.5 * acados_p_.p[l1_plus_l2] * w;
    v3 =  v + vn - 0.5 * acados_p_.p[l1_plus_l2] * w;
    v4 = -v + vn - 0.5 * acados_p_.p[l1_plus_l2] * w;
}

void NMPCNavControlOmni4::inverseKinematrics(const double v1, const double v2, const double v3, const double v4,
                                             double& v, double& vn, double& w)
{
    v  = ( v1 - v2 + v3 - v4) / 4.0;
    vn = (-v1 - v2 + v3 + v4) / 4.0;
    w  = (-v1 - v2 - v3 - v4) / (2.0 * acados_p_.p[l1_plus_l2]);
}

} // namespace nmpc_nav_control