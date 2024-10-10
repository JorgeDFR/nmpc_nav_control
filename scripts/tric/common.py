import numpy as np

# Tric AMR parameters
TAU_V = 0.1                            # [s] robot wheels velocity time constant
TAU_A = 0.5                            # [s] robot stearing angle time constant
DIST_D = 0.270                         # [m] distance between steering wheel and back wheels

TF_INI = 2.0                           # [s] desired prediction horizon
FREQ = 40                              # [Hz] controller frequency
DT = 1.0 / FREQ                        # [s] time step (normally the inverse of controller frequency)
N = int(np.ceil(TF_INI / DT))          # number of shooting nodes
TF = N * DT                            # [s] effective prediction horizon

V_MAX = 1.0                            # [m/s] maximum linear velocity
A_MAX = 1.0                            # [m/s^2] maximum acceleration
ALPHA_MIN = -30.0*(np.pi/180.0)        # [rad] minimum seeting angle
ALPHA_MAX =  30.0*(np.pi/180.0)        # [rad] maximum seeting angle
DALPHA_MAX = 2.0/3.0 * np.pi           # [rad/s] maximum seeting angular velocity

                                       # Path cost: state weights
Q = np.diag([10.0, 10.0, 5.0,          # robot pose (x, y, theta)
             0.0, 0.0,                 # robot linear velocity (v) and steering angle (alpha)
             0.0, 0.0])                # robot reference linear velocity (v_ref) and steering angle (alpha_ref)

                                       # Path cost: control weights
R = np.diag([1.0, 1.0])                # variation of reference velocity (dv_ref) and steering angle (dalpha_ref)

                                       # Terminal cost: state weights
QN = np.diag([10.0, 10.0, 5.0,         # robot pose (x, y, theta)
              0.0, 0.0,                # robot linear velocity (v) and steering angle (alpha)
              0.0, 0.0])               # robot reference linear velocity (v_ref) and steering angle (alpha_ref)