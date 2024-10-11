from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_omni4_amr_model(TAU_V, L1_PLUS_L2) -> AcadosModel:
    model_name = 'omni4amr'

    # states
    x = SX.sym('x')             # position x in map frame
    y = SX.sym('y')             # position y in map frame
    theta = SX.sym('theta')     # orientation theta in map frame
    v1 = SX.sym('v1')           # linear velocity wheel 1
    v2 = SX.sym('v2')           # linear velocity wheel 2
    v3 = SX.sym('v3')           # linear velocity wheel 3
    v4 = SX.sym('v4')           # linear velocity wheel 4
    v1_ref = SX.sym('v1_ref')   # reference linear velocity wheel 1
    v2_ref = SX.sym('v2_ref')   # reference linear velocity wheel 2
    v3_ref = SX.sym('v3_ref')   # reference linear velocity wheel 3
    v4_ref = SX.sym('v4_ref')   # reference linear velocity wheel 4
    sym_x = vertcat(x, y, theta,
                    v1, v2, v3, v4,
                    v1_ref, v2_ref, v3_ref, v4_ref)

    # controls
    dv1_ref = SX.sym('dv1_ref') # variation of the reference linear velocity wheel 1
    dv2_ref = SX.sym('dv2_ref') # variation of the reference linear velocity wheel 2
    dv3_ref = SX.sym('dv3_ref') # variation of the reference linear velocity wheel 3
    dv4_ref = SX.sym('dv4_ref') # variation of the reference linear velocity wheel 4
    sym_u = vertcat(dv1_ref, dv2_ref, dv3_ref, dv4_ref)

    # xdot for f_impl
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    theta_dot = SX.sym('theta_dot')
    v1_dot = SX.sym('v1_dot')
    v2_dot = SX.sym('v2_dot')
    v3_dot = SX.sym('v3_dot')
    v4_dot = SX.sym('v4_dot')
    v1_ref_dot = SX.sym('v1_ref_dot')
    v2_ref_dot = SX.sym('v2_ref_dot')
    v3_ref_dot = SX.sym('v3_ref_dot')
    v4_ref_dot = SX.sym('v4_ref_dot')
    sym_xdot = vertcat(x_dot, y_dot, theta_dot,
                       v1_dot, v2_dot, v3_dot, v4_dot,
                       v1_ref_dot, v2_ref_dot, v3_ref_dot, v4_ref_dot)

    # dynamics
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    v  = ( v1 - v2 + v3 - v4)/4.0
    vn = (-v1 - v2 + v3 + v4)/4.0
    w  = (-v1 - v2 - v3 - v4)/(2.0*L1_PLUS_L2)

    # dynamics
    x_dot      = v * cos_theta - vn * sin_theta
    y_dot      = v * sin_theta + vn * cos_theta
    theta_dot  = w
    v1_dot     = -1.0/TAU_V * v1 + 1.0/TAU_V * v1_ref
    v2_dot     = -1.0/TAU_V * v2 + 1.0/TAU_V * v2_ref
    v3_dot     = -1.0/TAU_V * v3 + 1.0/TAU_V * v3_ref
    v4_dot     = -1.0/TAU_V * v4 + 1.0/TAU_V * v4_ref
    v1_ref_dot = dv1_ref
    v2_ref_dot = dv2_ref
    v3_ref_dot = dv3_ref
    v4_ref_dot = dv4_ref

    f_expl = vertcat(x_dot, y_dot, theta_dot, 
                     v1_dot, v2_dot, v3_dot, v4_dot,
                     v1_ref_dot, v2_ref_dot, v3_ref_dot, v4_ref_dot)
    f_impl = sym_xdot - f_expl
    
    # model
    model = AcadosModel()
    model.name = model_name
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = sym_x
    model.xdot = sym_xdot
    model.u = sym_u

    # store meta information
    model.x_labels = ['$x$ [m]', '$y$ [m]', r'$\theta$ [rad]', 
                      '$v1$ [m/s]', '$v2$ [m/s]', '$v3$ [m/s]', '$v4$ [m/s]',
                      '$v1_{ref}$ [m/s]', '$v2_{ref}$ [m/s]', '$v3_{ref}$ [m/s]', '$v4_{ref}$ [m/s]']
    model.u_labels = ['$dv1_{ref}$ [m/s]', '$dv2_{ref}$ [m/s]', '$dv3_{ref}$ [m/s]', '$dv4_{ref}$ [m/s]']
    model.t_label = '$t$ [s]'

    return model