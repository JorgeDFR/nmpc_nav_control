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
        enum SystemStates {
            x = 0, y = 1, theta = 2,
            v = 3, alpha = 4,
            v_ref = 5, alpha_ref = 6
        };
        enum ControlInputs {
            dv_ref = 0, dalpha_ref = 1
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

        // Acados variables
        SolverInput acados_in_;
        SolverOutput acados_out_;
        tric3amr_solver_capsule* mpc_capsule_;

        // Other variables
        double dist_front_to_back_;
        double robot_steering_wheel_angle_;

    public:
        NMPCNavControlTric(double dt, double dist_back_to_front);
        ~NMPCNavControlTric() = default;

        double getHorizon() override { return TRIC3AMR_N; }

        void setSteeringWheelAngle(double robot_steering_wheel_angle) {
            robot_steering_wheel_angle_ = robot_steering_wheel_angle;
        }

        bool run(const Pose& robot_pose, const Vel& robot_vel,
                 const std::list<Pose>& traj_ref,
                 CmdVel& robot_vel_ref, double& cpu_time) override;

};

} // namespace nmpc_nav_control

#endif // NMPC_NAV_CONTROL_DIFF_H