import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from .common import load_parameters
from .omni4_amr_model import export_omni4_amr_model
from casadi import vertcat
from scipy.linalg import block_diag

def main(params):
    (N, TF, Q, R, QN, TAU_V, L1_PLUS_L2, V_MAX, A_MAX) = load_parameters(params)

    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_omni4_amr_model(TAU_V, L1_PLUS_L2)
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu  # y is x and u concatenated for compactness of the loss function
    
    # set prediction horizon
    ocp.solver_options.tf = TF
    ocp.solver_options.N_horizon = N

    # set path cost
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.W = block_diag(Q, R)

    # terminal cost
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr_e = model.x
    ocp.cost.yref_e = np.zeros((nx,))
    ocp.cost.W_e = QN

    # the 'EXTERNAL' cost type can be used to define general cost terms
    # NOTE: This leads to additional (exact) hessian contributions when using GAUSS_NEWTON hessian.
    
    # set constraints
    ocp.constraints.lbx = np.array([-V_MAX, -V_MAX, -V_MAX, -V_MAX])
    ocp.constraints.ubx = np.array([ V_MAX,  V_MAX,  V_MAX,  V_MAX])
    ocp.constraints.idxbx = np.array([7, 8, 9, 10])

    ocp.constraints.lbu = np.array([-A_MAX, -A_MAX, -A_MAX, -A_MAX])
    ocp.constraints.ubu = np.array([ A_MAX,  A_MAX,  A_MAX,  A_MAX])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    
    # initial state (will be overwritten later)
    ocp.constraints.x0 = np.array([0.0, 0.0, np.pi, 
                                   0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0])

    # reference trajectory (will be overwritten later)
    x_ref = np.zeros(nx)
    u_ref = np.zeros(nu)
    ocp.cost.yref = np.concatenate((x_ref, u_ref))
    ocp.cost.yref_e = x_ref

    # set options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
    # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI (Real Time Iteration), SQP

    solver_json = 'acados_ocp_' + model.name + '.json'
    ocp_solver = AcadosOcpSolver(ocp, json_file = solver_json, verbose=False)

    status = ocp_solver.solve()
    ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")

    if status != 0:
        raise Exception(f'acados returned status {status}.')