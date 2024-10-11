import numpy as np

# Load Omni4 AMR parameters
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
    L1_PLUS_L2 = params['l1_plus_l2']

    V_MAX = params['v_max']
    A_MAX = params['a_max']

    return (N, TF, Q, R, QN, TAU_V, L1_PLUS_L2, V_MAX, A_MAX)

# Omni4 AMR parameters
# TAU_V = 0.1                            # [s] robot wheels velocity time constant
# L1_PLUS_L2 = 0.270 + 0.265             # [m] total robot wheel distances: l1 -> front to back, l2 -> right to left

# TF_INI = 2.0                           # [s] desired prediction horizon
# FREQ = 40                              # [Hz] controller frequency
# DT = 1.0 / FREQ                        # [s] time step (normally the inverse of controller frequency)
# N = int(np.ceil(TF_INI / DT))          # number of shooting nodes
# TF = N * DT                            # [s] effective prediction horizon

# V_MAX = 1.0                            # [m/s] max linear velocity
# A_MAX = 1.0                            # [m/s^2] max acceleration

#                                        # Path cost: state weights
# Q = np.diag([10.0, 10.0, 10.0,         # robot pose (x, y, theta)
#              0.0, 0.0, 0.0, 0.0,       # robot wheel linear velocities (v1, v2, v3, v4)
#              0.0, 0.0, 0.0, 0.0])      # robot wheel reference velocities (v1_ref, v2_ref, v3_ref, v4_ref)

#                                        # Path cost: control weights
# R = np.diag([1.0, 1.0, 1.0, 1.0])      # variation of wheel reference velocities (dv1_ref, dv2_ref, dv3_ref, dv4_ref)

#                                        # Terminal cost: state weights
# QN = np.diag([10.0, 10.0, 10.0,        # robot pose (x, y, theta)
#               0.0, 0.0, 0.0, 0.0,      # robot wheel velocities (v1, v2, v3, v4)
#               0.0, 0.0, 0.0, 0.0])     # robot wheel reference velocities (v1_ref, v2_ref, v3_ref, v4_ref)
