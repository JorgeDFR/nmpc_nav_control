import numpy as np

# Load Tric AMR parameters
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
    TAU_A = params['tau_a']
    DIST_D = params['dist_d']

    V_MAX = params['v_max']
    A_MAX = params['a_max']
    ALPHA_MIN = params['alpha_min']*np.pi/180.0
    ALPHA_MAX = params['alpha_max']*np.pi/180.0
    DALPHA_MAX = params['dalpha_max']*np.pi/180.0

    return (N, TF, Q, R, QN, TAU_V, TAU_A, DIST_D, V_MAX, A_MAX, ALPHA_MIN, ALPHA_MAX, DALPHA_MAX)

# Tric AMR parameters
# TAU_V = 0.1                            # [s] robot wheels velocity time constant
# TAU_A = 0.5                            # [s] robot steering angle time constant
# DIST_D = 0.270                         # [m] distance between steering wheel and back wheels

# TF_INI = 2.0                           # [s] desired prediction horizon
# FREQ = 40                              # [Hz] controller frequency
# DT = 1.0 / FREQ                        # [s] time step (normally the inverse of controller frequency)
# N = int(np.ceil(TF_INI / DT))          # number of shooting nodes
# TF = N * DT                            # [s] effective prediction horizon

# V_MAX = 1.0                            # [m/s] maximum linear velocity
# A_MAX = 1.0                            # [m/s^2] maximum acceleration
# ALPHA_MIN = -30.0*(np.pi/180.0)        # [rad] minimum steering angle
# ALPHA_MAX =  30.0*(np.pi/180.0)        # [rad] maximum steering angle
# DALPHA_MAX = 2.0/3.0 * np.pi           # [rad/s] maximum steering angular velocity

#                                        # Path cost: state weights
# Q = np.diag([10.0, 10.0, 5.0,          # robot pose (x, y, theta)
#              0.0, 0.0,                 # robot linear velocity (v) and steering angle (alpha)
#              0.0, 0.0])                # robot reference linear velocity (v_ref) and steering angle (alpha_ref)

#                                        # Path cost: control weights
# R = np.diag([1.0, 1.0])                # variation of reference velocity (dv_ref) and steering angle (dalpha_ref)

#                                        # Terminal cost: state weights
# QN = np.diag([10.0, 10.0, 5.0,         # robot pose (x, y, theta)
#               0.0, 0.0,                # robot linear velocity (v) and steering angle (alpha)
#               0.0, 0.0])               # robot reference linear velocity (v_ref) and steering angle (alpha_ref)