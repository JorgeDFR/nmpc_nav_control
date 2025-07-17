import numpy as np

# Load Omni4 AMR parameters
def load_parameters(params):
    tf_ini = params['tf_ini']
    freq = params['freq']
    dt = 1.0 / freq
    N = int(np.ceil(tf_ini / dt))
    TF = N * dt

    L1_PLUS_L2 = params['l1_plus_l2']
    TAU_V = params['tau_v']

    V_MAX = params['v_max']
    A_MAX = params['a_max']

    Q_diag = params['Q_diag']
    R_diag = params['R_diag']
    QN_diag = params['QN_diag']

    return (N, TF, L1_PLUS_L2, TAU_V, V_MAX, A_MAX, Q_diag, R_diag, QN_diag)
