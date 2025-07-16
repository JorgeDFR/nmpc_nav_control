import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# System parameters
L = 0.5  # Wheelbase
nx = 5
nu = 2
Ts = 0.1
N = 20
Tf = Ts * N

# Dynamics
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
vL = ca.SX.sym('vL')
vR = ca.SX.sym('vR')
aL = ca.SX.sym('aL')
aR = ca.SX.sym('aR')

states = ca.vertcat(x, y, theta, vL, vR)
controls = ca.vertcat(aL, aR)

dx = (vL + vR)/2.0 * ca.cos(theta)
dy = (vL + vR)/2.0 * ca.sin(theta)
dtheta = (vR - vL)/L
dvL = aL
dvR = aR

rhs = ca.vertcat(dx, dy, dtheta, dvL, dvR)
f = ca.Function('f', [states, controls], [rhs])

# RK4 integrator
def rk4(xk, uk):
    k1 = f(xk, uk)
    k2 = f(xk + Ts/2 * k1, uk)
    k3 = f(xk + Ts/2 * k2, uk)
    k4 = f(xk + Ts * k3, uk)
    return xk + Ts/6 * (k1 + 2*k2 + 2*k3 + k4)

# Optimization problem
opti = ca.Opti()

X = opti.variable(nx, N+1)
U = opti.variable(nu, N)
X0_param = opti.parameter(nx)
REF = opti.parameter(3, N+1)  # Reference: x, y, theta

# Cost weights
Q = np.diag([10.0, 10.0, 5.0, 0.0, 0.0])
QN = Q
R = np.diag([1.0, 1.0])

cost = 0

for k in range(N):
    xk = X[:, k]
    uk = U[:, k]
    ref_k = REF[:, k]
    e = xk[:3] - ref_k
    cost += ca.mtimes([e.T, Q[:3,:3], e]) + ca.mtimes([uk.T, R, uk])
    x_next = rk4(xk, uk)
    opti.subject_to(X[:, k+1] == x_next)

# Terminal cost
eN = X[:3, -1] - REF[:, -1]
cost += ca.mtimes([eN.T, QN[:3,:3], eN])

opti.minimize(cost)

# Constraints
v_max = 1.0
a_max = 2.0

opti.subject_to(opti.bounded(-v_max, X[3:5, :], v_max))   # vL, vR
opti.subject_to(opti.bounded(-a_max, U, a_max))           # aL, aR
opti.subject_to(X[:, 0] == X0_param)                      # initial condition

# Solver setup
opts = {'ipopt.print_level': 0, 'print_time': False}
opti.solver('ipopt', opts)

# Simulation
sim_time = 5.0
n_steps = int(sim_time / Ts)
state_hist = []
u_hist = []

x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# Reference (constant pose)
xref = 2.0
yref = 0.0
thetaref = 0.0

for step in range(n_steps):
    # Set parameters
    opti.set_value(X0_param, x0)
    ref_traj = np.tile(np.array([[xref], [yref], [thetaref]]), (1, N+1))
    opti.set_value(REF, ref_traj)

    # Warm start
    if step > 0:
        opti.set_initial(X, X_sol)
        opti.set_initial(U, U_sol)

    sol = opti.solve()

    X_sol = sol.value(X)
    U_sol = sol.value(U)

    u0 = U_sol[:, 0]
    u_hist.append(u0)

    # Simulate dynamics
    x0 = rk4(x0, u0).full().flatten()
    state_hist.append(x0)

# Convert to arrays
state_hist = np.array(state_hist)
u_hist = np.array(u_hist)

# Plotting
plt.figure()
plt.scatter(xref, yref, c='r', label='Reference')
plt.scatter(state_hist[:, 0], state_hist[:, 1], c='blue', label='Trajectory')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("NMPC Trajectory Tracking (CasADi)")
plt.grid()
plt.axis('equal')
plt.legend()
plt.show()