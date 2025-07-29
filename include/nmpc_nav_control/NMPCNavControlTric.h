#ifndef NMPC_NAV_CONTROL_TRIC_H
#define NMPC_NAV_CONTROL_TRIC_H

#include "acados_solver_tric3amr.h"

#include "nmpc_nav_control/NMPCNavControl.h"

namespace nmpc_nav_control {

const std::string kTricStr = "tric";

class NMPCNavControlTric : public NMPCNavControl {
    public:
        struct CmdVelTric : public CmdVel {
            double v, alpha;
        };

    private:
        enum SystemStatesMap {
            x = 0, y = 1, theta = 2,
            v = 3, alpha = 4,
            v_ref = 5, alpha_ref = 6
        };
        enum ControlInputsMap {
            dv_ref = 0, dalpha_ref = 1
        };
        enum SystemParametersMap {
            dist_d = 0,
            tau_v = 1,
            tau_a = 2
        };
        struct SolverInput {
            double x0[TRIC3AMR_NX];
            double yref[TRIC3AMR_N+1][TRIC3AMR_NY];
        };
        struct SolverOutput {
            double u0[TRIC3AMR_NU];
            double x1[TRIC3AMR_NX];
            double status, kkt_res, cpu_time;
        };
        struct SolverParameters {
            double p[TRIC3AMR_NP];
            double x_min[TRIC3AMR_NBX];
            double x_max[TRIC3AMR_NBX];
            double u_min[TRIC3AMR_NBU];
            double u_max[TRIC3AMR_NBU];
            double W[TRIC3AMR_NY*TRIC3AMR_NY];
            double W_e[TRIC3AMR_NYN*TRIC3AMR_NYN];
        };

        // Acados variables
        SolverInput acados_in_;
        SolverOutput acados_out_;
        SolverParameters acados_p_;
        tric3amr_solver_capsule* mpc_capsule_;

        // Other variables
        double robot_steering_wheel_angle_;

    public:
        NMPCNavControlTric(double dt, double dist_d, double tau_v, double tau_a, double v_max, double a_max,
                           double alpha_min, double alpha_max, double dalpha_max, std::vector<double> W_diag);
        ~NMPCNavControlTric();

        double getHorizon() override { return TRIC3AMR_N; }

        void setSteeringWheelAngle(double robot_steering_wheel_angle) {
            robot_steering_wheel_angle_ = robot_steering_wheel_angle;
        }

        bool run(const Pose& robot_pose, const Vel& robot_vel,
                 const std::list<Pose>& traj_ref,
                 CmdVel& robot_vel_ref, double& cpu_time) override;

        bool reset_mpc() override;
};

} // namespace nmpc_nav_control

#endif // NMPC_NAV_CONTROL_DIFF_H