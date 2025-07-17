#ifndef NMPC_NAV_CONTROL_DIFF_H
#define NMPC_NAV_CONTROL_DIFF_H

#include "acados_solver_diff2amr.h"

#include "nmpc_nav_control/NMPCNavControl.h"

namespace nmpc_nav_control {

const std::string kDiffStr = "diff";

class NMPCNavControlDiff : public NMPCNavControl {
    public:
        struct CmdVelDiff : public CmdVel {
            double v, w;
        };

    private:
        enum SystemStatesMap {
            x = 0, y = 1, theta = 2,
            vl = 3, vr = 4,
            vl_ref = 5, vr_ref = 6
        };
        enum ControlInputsMap {
            dvl_ref = 0, dvr_ref = 1
        };
        enum SystemParametersMap {
            dist_b = 0,
            tau_v = 1
        };
        struct SolverInput {
            double x0[DIFF2AMR_NX];
            double yref[DIFF2AMR_N+1][DIFF2AMR_NY];
        };
        struct SolverOutput {
            double u0[DIFF2AMR_NU];
            double x1[DIFF2AMR_NX];
            double status, kkt_res, cpu_time;
        };
        struct SolverParameters {
            double p[DIFF2AMR_NP];
            double x_min[DIFF2AMR_NBX];
            double x_max[DIFF2AMR_NBX];
            double u_min[DIFF2AMR_NBU];
            double u_max[DIFF2AMR_NBU];
            double W[DIFF2AMR_NY*DIFF2AMR_NY];
            double W_e[DIFF2AMR_NYN*DIFF2AMR_NYN];
        };

        // Acados variables
        SolverInput acados_in_;
        SolverOutput acados_out_;
        SolverParameters acados_p_;
        diff2amr_solver_capsule* mpc_capsule_;

    public:
        NMPCNavControlDiff(double dt, double dist_b, double tau_v, double v_max, double a_max);
        ~NMPCNavControlDiff();

        double getHorizon() override { return DIFF2AMR_N; }

        bool run(const Pose& robot_pose, const Vel& robot_vel,
                 const std::list<Pose>& traj_ref,
                 CmdVel& robot_vel_ref, double& cpu_time) override;

    private:
        void directKinematrics(const double v, const double w, double& vl, double& vr);

        void inverseKinematrics(const double vl, const double vr, double& v, double& w);

};

} // namespace nmpc_nav_control

#endif // NMPC_NAV_CONTROL_DIFF_H