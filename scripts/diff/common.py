import numpy as np

# Load Diff AMR parameters
def load_parameters(params):
    tf_ini = params['tf_ini']
    freq = params['freq']
    dt = 1.0 / freq                       
    N = int(np.ceil(tf_ini / dt))
    TF = N * dt

    Q = np.diag(params['Q'])
    R = np.diag(params['R'])
    QN = np.diag(params['QN'])

    TAU_V = params['tau_v']                           
    DIST_B = params['dist_b']

    V_MAX = params['v_max']
    A_MAX = params['a_max']

    return (N, TF, Q, R, QN, TAU_V, DIST_B, V_MAX, A_MAX)

# Diff AMR parameters
# TAU_V = 0.1                            # [s] robot wheels velocity time constant
# DIST_B = 0.270                         # [m] distance between left and right wheels

# TF_INI = 2.0                           # [s] desired prediction horizon
# FREQ = 40                              # [Hz] controller frequency
# DT = 1.0 / FREQ                        # [s] time step (normally the inverse of controller frequency)
# N = int(np.ceil(TF_INI / DT))          # number of shooting nodes
# TF = N * DT                            # [s] effective prediction horizon

# V_MAX = 1.0                            # [m/s] max linear velocity
# A_MAX = 1.0                            # [m/s^2] max acceleration

#                                        # Path cost: state weights
# Q = np.diag([10.0, 10.0, 5.0,          # robot pose (x, y, theta)
#              0.0, 0.0,                 # robot wheel linear velocities (vl, vr) -> (left, right)
#              0.0, 0.0])                # robot wheel reference velocities (vl_ref, vr_ref) -> (left, right)

#                                        # Path cost: control weights
# R = np.diag([1.0, 1.0])                # variation of wheel reference velocities (dvl_ref, dvr_ref)

#                                        # Terminal cost: state weights
# QN = np.diag([10.0, 10.0, 5.0,         # robot pose (x, y, theta)
#               0.0, 0.0,                # robot wheel velocities (vl, vr)
#               0.0, 0.0])               # robot wheel reference velocities (vl_ref, vr_ref)
