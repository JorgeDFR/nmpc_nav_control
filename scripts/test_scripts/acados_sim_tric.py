import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from scipy.linalg import block_diag

# Parameters
DIST_D = 0.2  # Distance to steer wheel [m]

# State and control variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
v = ca.SX.sym('v')
alpha = ca.SX.sym('alpha')
dv = ca.SX.sym('dv')
dalpha = ca.SX.sym('dalpha')

sym_x = ca.vertcat(x, y, theta, v, alpha)
sym_u = ca.vertcat(dv, dalpha)

# Dynamics
x_dot = ca.SX.sym('x_dot')
y_dot = ca.SX.sym('y_dot')
theta_dot = ca.SX.sym('theta_dot')
v_dot = ca.SX.sym('v_dot')
alpha_dot = ca.SX.sym('alpha_dot')
sym_xdot = ca.vertcat(x_dot, y_dot, theta_dot, v_dot, alpha_dot)

x_dot = v * ca.cos(theta) * ca.cos(alpha)
y_dot = v * ca.sin(theta) * ca.cos(alpha)
theta_dot =  v/DIST_D * ca.sin(alpha)
v_dot = dv
alpha_dot = dalpha
f_expl = ca.vertcat(x_dot, y_dot, theta_dot, v_dot, alpha_dot)
f_impl = sym_xdot - f_expl

# Create Acados model
acados_model = AcadosModel()
acados_model.name = 'tricicle'
acados_model.x = sym_x
acados_model.xdot = sym_xdot
acados_model.u = sym_u
acados_model.f_expl_expr = f_expl
acados_model.f_impl_expr = f_impl

# Create OCP
ocp = AcadosOcp()
ocp.model = acados_model

# Discretization
Ts = 0.025   # Sampling time
N  = 80      # Prediction horizon
Tf = N * Ts  # Time horizon

ocp.solver_options.N_horizon = N
ocp.solver_options.tf = Tf

# Cost function
Q  = np.diag([10.0, 10.0, 5.0, 0.0, 0.0])  # State cost weights
QN = np.diag([1000.0, 1000.0, 500.0, 0.0, 0.0])  # State terminal cost weights
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
a_max = 1.0  # Max wheel acceleration [m/s^2]
alpha_min = -np.pi/3  # Min steer wheel angle [rad]
alpha_max =  np.pi/3  # Max steer wheel angle [rad]
dalpha_max = 0.5  # Max steer wheel angular variation [rad/s]


ocp.constraints.lbx   = np.array([-v_max, alpha_min])
ocp.constraints.ubx   = np.array([ v_max, alpha_max])
ocp.constraints.idxbx = np.array([3, 4])

ocp.constraints.lbu   = np.array([-a_max, -dalpha_max])
ocp.constraints.ubu   = np.array([ a_max,  dalpha_max])
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
sim_time = 10.0  # Total simulation time [s]
n_steps = int(sim_time / (Tf / N))
state_history = np.zeros((n_steps+1, 5))
state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Initial state

# Time vector
t = np.linspace(0.0, sim_time, n_steps+1)

# Reference trajectory (Pose)
x_ref = np.full_like(t, 0.1)
y_ref = np.full_like(t, 2.0)
theta_ref = np.full_like(t, 0.0)

state_history[0, :] = state
state_ocp = state
for i in range(n_steps):
    # Set current state
    ocp_solver.set(0, 'lbx', state_ocp)
    ocp_solver.set(0, 'ubx', state_ocp)

    # Set references
    for j in range(N):
        index = min(i+j, n_steps)
        yref = np.array([x_ref[index], y_ref[index], theta_ref[index], 0.0, 0.0, 0.0, 0.0])
        ocp_solver.set(j, 'yref', yref)

    # Terminal reference
    index = min(i+N, n_steps)
    yref_e = np.array([x_ref[index], y_ref[index], theta_ref[index], 0.0, 0.0])
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
    theta = state[2]
    v = state[3]
    alpha = state[4]
    dx = v * np.cos(theta) * np.cos(alpha)
    dy = v * np.sin(theta) * np.cos(alpha)
    dtheta = v/DIST_D * np.sin(alpha)
    dvL = u[0] + np.random.normal(0, 0.05)
    dvR = u[1] + np.random.normal(0, 0.05)

    state[0] += dx * dt
    state[1] += dy * dt
    state[2] += dtheta * dt
    state[3] += dvL * dt
    state[4] += dvR * dt

    state_history[i+1, :] = state

    state_ocp = state

import matplotlib.animation as animation
from matplotlib.patches import Polygon, Rectangle

ROBOT_WIDTH = 0.2
STEER_WHEEL_LENGTH = 0.1
STEER_WHEEL_WIDTH = 0.02

def create_robot_patch(x, y, theta, alpha):
    """
    Creates a triangular patch (robot body) and a rectangular patch (steering wheel).
    """
    # Robot triangle
    width = ROBOT_WIDTH
    length = DIST_D

    # Define triangle (body) in local frame
    triangle = np.array([
        [length, 0],           # Front tip
        [-length/2, -width/2], # Rear left
        [-length/2, width/2],  # Rear right
    ])

    # Rotation matrix for theta
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    # Rotate and translate triangle
    triangle = (R @ triangle.T).T + np.array([x, y])

    # Steering wheel rectangle (placed at front tip of triangle)
    steer_length = STEER_WHEEL_LENGTH
    steer_width = STEER_WHEEL_WIDTH

    # Steering frame rotation
    steer_R = np.array([
        [np.cos(theta + alpha), -np.sin(theta + alpha)],
        [np.sin(theta + alpha),  np.cos(theta + alpha)]
    ])

    # Rectangle origin (bottom-left corner)
    steer_center = np.array([x, y]) + R @ np.array([length, 0])
    steer_rect = Rectangle(
        steer_center - steer_R @ np.array([steer_length/2, steer_width/2]),
        steer_length,
        steer_width,
        angle=np.degrees(theta + alpha),
        color='red'
    )

    return triangle, steer_rect

# Set up figure
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_aspect('equal')
ax.set_xlim(np.min(state_history[:, 0]) - 1, np.max(state_history[:, 0]) + 1)
ax.set_ylim(np.min(state_history[:, 1]) - 1, np.max(state_history[:, 1]) + 1)
ax.set_title('Robot Motion Animation')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.grid()

# Initialize elements
robot_patch = Polygon([[0, 0]], closed=True, color='blue', label='Robot')
steer_patch = Rectangle((0, 0), STEER_WHEEL_LENGTH, STEER_WHEEL_WIDTH, color='red', label='Steer Wheel')
ref_traj, = ax.plot([], [], 'ro', markersize=3, label='Reference Pose')
robot_traj, = ax.plot([], [], 'k--', linewidth=1.5, label='Robot Trajectory')

ax.add_patch(robot_patch)
ax.add_patch(steer_patch)
ax.legend()

def init():
    robot_patch.set_xy([[0, 0]])
    steer_patch.set_xy([0, 0])
    ref_traj.set_data([], [])
    robot_traj.set_data([], [])
    return robot_patch, steer_patch, ref_traj, robot_traj

def animate(i):
    x, y, theta, v, alpha = state_history[i, :]
    triangle, steer_rect = create_robot_patch(x, y, theta, alpha)

    # Update patches
    robot_patch.set_xy(triangle)
    steer_patch.set_xy(steer_rect.get_xy())
    steer_patch.angle = steer_rect.angle

    # Update reference pose
    ref_traj.set_data(x_ref[:i+1], y_ref[:i+1])

    # Update trajectory
    robot_traj.set_data(state_history[:i+1, 0], state_history[:i+1, 1])

    return robot_patch, steer_patch, ref_traj, robot_traj

# Animate
ani = animation.FuncAnimation(fig, animate, frames=len(t), init_func=init,
                              interval=Ts*1000, blit=True, repeat=False)

plt.show()

plt.figure()
plt.scatter(x_ref, y_ref, c='r', label='Reference')
plt.scatter(state_history[:, 0], state_history[:, 1], c='b', label='Trajectory')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('NMPC Trajectory Tracking (acados)')
plt.legend()
plt.grid()
plt.axis('equal')

plt.figure()
plt.plot(t, state_history[:, 0], label='x')
plt.plot(t, state_history[:, 1], label='y')
plt.plot(t, state_history[:, 2], label='theta')
plt.plot(t, state_history[:, 3], label='v')
plt.plot(t, state_history[:, 4], label='alpha')
plt.xlabel('time')
plt.ylabel('states')
plt.title('States over time')
plt.legend()
plt.grid()
plt.axis('equal')

plt.show()