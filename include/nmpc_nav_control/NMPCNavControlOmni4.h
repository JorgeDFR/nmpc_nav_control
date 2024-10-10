#ifndef NMPC_NAV_CONTROL_OMNI4_H
#define NMPC_NAV_CONTROL_OMNI4_H

#include "omni4amr_model/omni4amr_model.h"
#include "acados_solver_omni4amr.h"

#include "nmpc_nav_control/NMPCNavControl.h"

namespace nmpc_nav_control {

class NMPCNavControlOmni4 : public NMPCNavControl {
    public:
        struct CmdVelOmni4 : public CmdVel {
            double v, vn, w;
        };

    private:
        enum SystemStates {
            x = 0, y = 1, theta = 2,
            v1 = 3, v2 = 4, v3 = 5, v4 = 6,
            v1_ref = 7, v2_ref = 8, v3_ref = 9, v4_ref = 10
        };
        enum ControlInputs {
            dv1_ref = 0, dv2_ref = 1, dv3_ref = 2, dv4_ref = 3
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
        
        // Acados variables
        SolverInput acados_in_;
        SolverOutput acados_out_;
        omni4amr_solver_capsule* mpc_capsule_;

        // Other variables
        double l1_plus_l2_;

    public:
        NMPCNavControlOmni4(double dt, double l1_plus_l2);
        ~NMPCNavControlOmni4() = default;

        double getHorizon() override { return OMNI4AMR_N; }

        bool run(const Pose& robot_pose, const std::list<Pose>& traj_ref, 
                 CmdVel& robot_vel_ref, double& cpu_time) override;

    private:
        void directKinematrics(const double v, const double vn, const double w, 
                               double& v1, double& v2, double& v3, double& v4);

        void inverseKinematrics(const double v1, const double v2, const double v3, const double v4,
                                double& v, double& vn, double& w);

};

} // namespace nmpc_nav_control

#endif // NMPC_NAV_CONTROL_OMNI4_H