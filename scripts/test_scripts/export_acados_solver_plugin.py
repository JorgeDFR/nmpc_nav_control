import sys

from acados_template import AcadosOcp
from casadi import SX, vertcat, diag
import numpy as np

from diff2amr_model import Diff2AmrModel

def export_acados_ocp() -> AcadosOcp:
    # default state and control bounds
    v_max = 1.0 # m/s

    # setup base robot model
    omni4amr_model = Diff2AmrModel()
    model = omni4amr_model.export_acados_model()

    # declare extra parameters
    sym_p_ref = SX.sym('p_ref', 3)
    sym_Q_diag = SX.sym('Q_diag', 3)
    sym_QN_diag = SX.sym('QN_diag', 3)
    sym_R_diag = SX.sym('R_diag', 2)

    model.p = vertcat(
        model.p,         # original model parameters (see omni4amr_model.py)
        sym_p_ref,       # reference position
        sym_Q_diag,      # weight for state error
        sym_QN_diag,     # weight for state terminal cost error
        sym_R_diag,      # weight for control
    )

    # setup OCP
    Tf_ini = 2.0                  # desired prediction horizon [s]
    freq = 40.0                   # controller frequency [Hz]
    dt = 1.0 / freq               # delta time [s]
    N = int(np.ceil(Tf_ini / dt)) # number of sampling periods
    Tf = N * dt                   # prediction horizon length [s]

    ocp = AcadosOcp()
    ocp.model = model
    ocp.dims.N = N

    # set default parameter values
    ocp.parameter_values = np.zeros((model.p.shape[0],))

    # set cost matrices
    Q = diag(sym_Q_diag)
    QN = diag(sym_QN_diag)
    R = diag(sym_R_diag)

    # set path cost
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.cost.yref = vertcat(sym_p_ref, SX.zeros(2))
    ocp.cost.W = diag(vertcat(sym_Q_diag, sym_R_diag))

    # terminal cost
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr_e = model.x
    ocp.cost.yref_e = vertcat(sym_p_ref)
    ocp.cost.W_e = QN

    # the 'EXTERNAL' cost type can be used to define general cost terms
    # NOTE: This leads to additional (exact) hessian contributions when using GAUSS_NEWTON hessian.

    # set initial state constraint
    ocp.constraints.x0 = np.zeros((model.x.shape[0],))

    # set state constraints
    # ocp.constraints.lbx = np.array([])
    # ocp.constraints.ubx = np.array([])
    # ocp.constraints.idxbx = np.array([0, 1, 2])

    # set terminal state constraints
    # ocp.constraints.lbx_e = ocp.constraints.lbx
    # ocp.constraints.ubx_e = ocp.constraints.ubx
    # ocp.constraints.idxbx_e = ocp.constraints.idxbx

    # set control constraints
    ocp.constraints.lbu = np.array([-v_max, -v_max])
    ocp.constraints.ubu = np.array([ v_max,  v_max])
    ocp.constraints.idxbu = np.array([0, 1])

    # set solver options
    ocp.solver_options.tf = Tf
    ocp.solver_options.N_horizon = N

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
    # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP

    ocp.solver_options.hessian_approx = 'EXACT' # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = 'IRK' # IRK, ERK
    ocp.solver_options.hpipm_mode = 'BALANCE'  # 'SPEED', 'ROBUST', 'BALANCE'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI (Real Time Iteration), SQP
    ocp.solver_options.qp_solver_warm_start = 1  # 0 = None, 1 = warm, 2 = hot
    ocp.solver_options.qp_solver_iter_max = 50

    return ocp

def main() -> int:
    # Create ocp
    acados_ocp = export_acados_ocp()

    # Define the index maps
    x_index_map = {
        'x': [0],
        'y': [1],
        'theta': [2],
    }
    p_index_map = {
        'dist_b': [0],
        'p_ref': [1, 2, 3],
        'Q_diag': [4, 5, 6],
        'QN_diag': [7, 8, 9],
        'R_diag': [10, 11],
    }
    u_index_map = {
        'vr': [0],
        'vl': [1],
    }

    # Instantiate plugin generator
    solver_plugin_generator = SolverPluginGenerator()

    solver_plugin_generator.generate_solver_plugin(
        acados_ocp,
        plugin_class_name='Diff2Amr',
        solver_description='Acados solver plugin to control a differential steering robot',
        x_index_map=x_index_map,
        p_index_map=p_index_map,
        u_index_map=u_index_map,
    )

    return 0


if __name__ == '__main__':
    sys.exit(main())
