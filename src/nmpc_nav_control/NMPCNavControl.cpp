#include "nmpc_nav_control/NMPCNavControl.h"

namespace nmpc_nav_control {

void NMPCNavControl::processCreateStatus(const int create_status)
{
    if (create_status != 0) {
        std::stringstream error;
        error << "acados_create() returned status " << create_status << ".";
        throw std::runtime_error(error.str());
    }
}

bool NMPCNavControl::processAcadosStatus(const int acados_status)
{
    if (acados_status != 0) {
        std::stringstream error;
        error << "acados_solve() returned status " << acados_status << ".";
        throw std::runtime_error(error.str());
        return false;
    }
    return true;
}

double NMPCNavControl::unwrapAngle(double current, double previous) 
{
    double delta = current - previous;
    if (delta > M_PI) { current -= 2 * M_PI; } 
    else if (delta < -M_PI) { current += 2 * M_PI; }
    return current;
}

} // namespace nmpc_nav_control