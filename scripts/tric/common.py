import numpy as np

# Load Tric AMR parameters
def load_parameters(params):
    tf_ini = params['tf_ini']
    freq = params['freq']
    dt = 1.0 / freq
    N = int(np.ceil(tf_ini / dt))
    TF = N * dt

    DIST_D = params['dist_d']
    TAU_V = params['tau_v']
    TAU_A = params['tau_a']

    V_MAX = params['v_max']
    A_MAX = params['a_max']
    ALPHA_MIN = params['alpha_min']*np.pi/180.0
    ALPHA_MAX = params['alpha_max']*np.pi/180.0
    DALPHA_MAX = params['dalpha_max']*np.pi/180.0

    Q_diag = params['Q_diag']
    R_diag = params['R_diag']
    QN_diag = params['QN_diag']

    return (N, TF, DIST_D, TAU_V, TAU_A, V_MAX, A_MAX, ALPHA_MIN, ALPHA_MAX, DALPHA_MAX, Q_diag, R_diag, QN_diag)