#ifndef NMPC_NAV_CONTROL_H
#define NMPC_NAV_CONTROL_H

#include <stdexcept>
#include <sstream>

#include <itrci_nav/ParametricPathSet.h>
#include <itrci_nav/ParametricPath.h>

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

namespace nmpc_nav_control {

class NMPCNavControl {
    public:
        struct Pose {
            double x, y, theta;
        };
        struct Vel {
            double v, vn, w;
        };
        struct CmdVel {
            virtual ~CmdVel() = default;
        };

    protected:
        double dt_;

    public:
        NMPCNavControl(double dt) : dt_(dt) {}
        virtual ~NMPCNavControl() = default;

        virtual double getHorizon() = 0;
        double getDeltaTime() { return dt_; }

        virtual bool run(const Pose& robot_pose, const Vel& robot_vel,
                         const std::list<Pose>& traj_ref,
                         CmdVel& robot_vel_ref, double& cpu_time) = 0;

        // Attempt to cast to the specific command velocity type
        template<typename T>
        T* getCommandVelocity(CmdVel& cmd_vel) {
            return dynamic_cast<T*>(&cmd_vel);
        }

    protected:
        void processCreateStatus(const int create_status);
        bool processAcadosStatus(const int acados_status);
        double unwrapAngle(double current, double previous);
};

} // namespace nmpc_nav_control

#endif // NMPC_NAV_CONTROL_H