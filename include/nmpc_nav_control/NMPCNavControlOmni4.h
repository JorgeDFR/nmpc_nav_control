#ifndef NMPC_NAV_CONTROL_OMNI4_H
#define NMPC_NAV_CONTROL_OMNI4_H

#include "acados_solver_omni4amr.h"

#include "nmpc_nav_control/NMPCNavControl.h"

namespace nmpc_nav_control {

const std::string kOmni4Str = "omni4";

class NMPCNavControlOmni4 : public NMPCNavControl {
    public:
        struct CmdVelOmni4 : public CmdVel {
            double v, vn, w;
        };

    private:
        enum SystemStatesMap {
            x = 0, y = 1, theta = 2,
            v1 = 3, v2 = 4, v3 = 5, v4 = 6,
            v1_ref = 7, v2_ref = 8, v3_ref = 9, v4_ref = 10
        };
        enum ControlInputsMap {
            dv1_ref = 0, dv2_ref = 1, dv3_ref = 2, dv4_ref = 3
        };
        enum SystemParametersMap {
            l1_plus_l2 = 0,
            tau_v = 1
        };
        struct SolverInput {
            double x0[OMNI4AMR_NX];
            double yref[OMNI4AMR_N+1][OMNI4AMR_NY];
        };
        struct SolverOutput {
            double u0[OMNI4AMR_NU];
            double x1[OMNI4AMR_NX];
            double status, kkt_res, cpu_time;
        };
        struct SolverParameters {
            double p[OMNI4AMR_NP];
            double x_min[OMNI4AMR_NBX];
            double x_max[OMNI4AMR_NBX];
            double u_min[OMNI4AMR_NBU];
            double u_max[OMNI4AMR_NBU];
            double W[OMNI4AMR_NY*OMNI4AMR_NY];
            double W_e[OMNI4AMR_NYN*OMNI4AMR_NYN];
        };

        // Acados variables
        SolverInput acados_in_;
        SolverOutput acados_out_;
        SolverParameters acados_p_;
        omni4amr_solver_capsule* mpc_capsule_;

    public:
        NMPCNavControlOmni4(double dt, double l1_plus_l2, double tau_v, double v_max, double a_max, std::vector<double> W_diag);
        ~NMPCNavControlOmni4();

        double getHorizon() override { return OMNI4AMR_N; }

        bool run(const Pose& robot_pose, const Vel& robot_vel,
                 const std::list<Pose>& traj_ref,
                 CmdVel& robot_vel_ref, double& cpu_time) override;

        bool reset_mpc() override;

    private:
        void directKinematrics(const double v, const double vn, const double w,
                               double& v1, double& v2, double& v3, double& v4);

        void inverseKinematrics(const double v1, const double v2, const double v3, const double v4,
                                double& v, double& vn, double& w);

};

} // namespace nmpc_nav_control

#endif // NMPC_NAV_CONTROL_OMNI4_H