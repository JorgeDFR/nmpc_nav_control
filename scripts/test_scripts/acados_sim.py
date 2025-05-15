import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from scipy.linalg import block_diag

# Parameters
L = 0.5  # Wheelbase [m]

# State and control variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
vL = ca.SX.sym('vL')
vR = ca.SX.sym('vR')
aL = ca.SX.sym('aL')
aR = ca.SX.sym('aR')

sym_x = ca.vertcat(x, y, theta, vL, vR)
sym_u = ca.vertcat(aL, aR)

# Dynamics
x_dot = ca.SX.sym('x_dot')
y_dot = ca.SX.sym('y_dot')
theta_dot = ca.SX.sym('theta_dot')
vL_dot = ca.SX.sym('vL_dot')
vR_dot = ca.SX.sym('vR_dot')
sym_xdot = ca.vertcat(x_dot, y_dot, theta_dot, vL_dot, vR_dot)

x_dot = (vL + vR)/2.0 * ca.cos(theta)
y_dot = (vL + vR)/2.0 * ca.sin(theta)
theta_dot = (vR - vL)/L
vL_dot = aL
vR_dot = aR

f_expl = ca.vertcat(x_dot, y_dot, theta_dot, vL_dot, vR_dot)
f_impl = sym_xdot - f_expl

# Create Acados model
acados_model = AcadosModel()
acados_model.name = 'diff_drive'
acados_model.x = sym_x
acados_model.xdot = sym_xdot
acados_model.u = sym_u
acados_model.f_expl_expr = f_expl
acados_model.f_impl_expr = f_impl
acados_model.p = ca.SX.sym('p', 0)

# Create OCP
ocp = AcadosOcp()
ocp.model = acados_model

# Discretization
Ts = 0.1     # Sampling time
N  = 20      # Prediction horizon
Tf = N * Ts  # Time horizon

ocp.dims.N = N
ocp.solver_options.tf = Tf

# Cost function
Q  = np.diag([10.0, 10.0, 5.0, 6.0, 6.0])  # State cost weights
QN = np.diag([10.0, 10.0, 5.0, 6.0, 6.0])  # State terminal cost weights
R  = np.diag([1.0, 1.0])                   # Control cost weights

nx = acados_model.x.rows()
nu = acados_model.u.rows()
ny = nx + nu 

ocp.cost.cost_type = 'NONLINEAR_LS'
ocp.model.cost_y_expr = ca.vertcat(acados_model.x, acados_model.u)
ocp.cost.yref = np.zeros((ny,))
ocp.cost.W = block_diag(Q, R)

ocp.cost.cost_type_e = 'NONLINEAR_LS'
ocp.model.cost_y_expr_e = acados_model.x
ocp.cost.yref_e = np.zeros((nx,))
ocp.cost.W_e = QN

# Constraints
v_max = 1.0  # Max wheel velocity [m/s]
a_max = 2.0  # Max wheel acceleration [m/s^2]

ocp.constraints.lbx   = np.array([-v_max, -v_max])
ocp.constraints.ubx   = np.array([ v_max,  v_max])
ocp.constraints.idxbx = np.array([3, 4])

ocp.constraints.lbu   = np.array([-a_max, -a_max])
ocp.constraints.ubu   = np.array([ a_max,  a_max])
ocp.constraints.idxbu = np.array([0, 1])

ocp.constraints.x0    = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # Initial state (will be overwrited after)

# Solver options
ocp.solver_options.qp_solver       = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx  = 'GAUSS_NEWTON'  # 'GAUSS_NEWTON', 'EXACT'
ocp.solver_options.integrator_type = 'ERK'  # IRK, ERK
ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # SQP_RTI (Real Time Iteration), SQP

# Create solver
ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

# Simulation parameters
sim_time = 5.0  # Total simulation time [s]
n_steps = int(sim_time / (Tf / N))
state_history = np.zeros((n_steps+1, 5))
state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Initial state

# Time vector
t = np.linspace(0.0, sim_time, n_steps+1)

# Reference trajectory (Line)
# v_ref = 0.5  # Reference speed [m/s]
# x_ref = 0.0 + v_ref * t
# y_ref = 0.0 + v_ref * t
# theta_ref = np.pi/4 * np.ones_like(t)

# Reference trajectory (Pose)
x_ref = np.full_like(t, 2.0)
y_ref = np.full_like(t, 0.0)
theta_ref = np.full_like(t, 0.0)

state_history[0, :] = state

for i in range(n_steps):
    # Set current state
    ocp_solver.set(0, 'lbx', state)
    ocp_solver.set(0, 'ubx', state)

    # Set references
    for j in range(N):
        index = min(i+j, n_steps)
        yref = np.array([x_ref[index], y_ref[index], theta_ref[index], 0.0, 0.0, 0.0, 0.0])
        ocp_solver.set(j, 'yref', yref)

    # Terminal reference
    yref_e = np.array([x_ref[-1], y_ref[-1], theta_ref[-1], 0.0, 0.0])
    ocp_solver.set(N, 'yref', yref_e)

    # Solve OCP
    status = ocp_solver.solve()
    if status != 0:
        print(f"OCP solver failed at step {i} with status {status}")
        break

    # Apply control input
    u = ocp_solver.get(0, 'u')
    dt = Tf / N
    
    # Simple Euler integration
    vL = state[3]
    vR = state[4]
    theta = state[2]
    dx = (vL + vR)/2 * np.cos(theta)
    dy = (vL + vR)/2 * np.sin(theta)
    dtheta = (vR - vL)/L
    dvL = u[0]
    dvR = u[1]

    state[0] += dx * dt
    state[1] += dy * dt
    state[2] += dtheta * dt
    state[3] += dvL * dt
    state[4] += dvR * dt

    state_history[i+1, :] = state

plt.figure()
plt.scatter(x_ref, y_ref, c='r', label='Reference')
plt.scatter(state_history[:, 0], state_history[:, 1], c='b', label='Trajectory')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('NMPC Trajectory Tracking (acados)')
plt.legend()
plt.grid()
plt.axis('equal')
plt.show()
