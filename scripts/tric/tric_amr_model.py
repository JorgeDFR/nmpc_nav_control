import casadi as ca
from acados_template import AcadosModel

def export_tric_amr_model() -> AcadosModel:
    model_name = 'tric3amr'

    # states
    x = ca.SX.sym('x')                   # position x in map frame
    y = ca.SX.sym('y')                   # position y in map frame
    theta = ca.SX.sym('theta')           # orientation theta in map frame
    v = ca.SX.sym('v')                   # linear velocity
    alpha = ca.SX.sym('alpha')           # steering angle
    v_ref = ca.SX.sym('v_ref')           # reference linear velocity
    alpha_ref = ca.SX.sym('alpha_ref')   # reference steering angle
    sym_x = ca.vertcat(x, y, theta,
                       v, alpha,
                       v_ref, alpha_ref)

    # controls
    dv_ref = ca.SX.sym('dv_ref')         # variation of the reference linear velocity
    dalpha_ref = ca.SX.sym('dalpha_ref') # variation of the reference steering angle
    sym_u = ca.vertcat(dv_ref, dalpha_ref)

    # parameters
    dist_d = ca.SX.sym('dist_d')
    tau_v = ca.SX.sym('tau_v')
    tau_a = ca.SX.sym('tau_a')
    sym_p = ca.vertcat(dist_d, tau_v, tau_a)

    # xdot for f_impl
    x_dot = ca.SX.sym('x_dot')
    y_dot = ca.SX.sym('y_dot')
    theta_dot = ca.SX.sym('theta_dot')
    v_dot = ca.SX.sym('v_dot')
    alpha_dot = ca.SX.sym('alpha_dot')
    v_ref_dot = ca.SX.sym('v_ref_dot')
    alpha_ref_dot = ca.SX.sym('alpha_ref_dot')
    sym_xdot = ca.vertcat(x_dot, y_dot, theta_dot,
                          v_dot, alpha_dot,
                          v_ref_dot, alpha_ref_dot)

    # dynamics
    cos_theta = ca.cos(theta)
    sin_theta = ca.sin(theta)
    cos_alpha = ca.sin(alpha)
    sin_alpha = ca.sin(alpha)

    # dynamics
    x_dot         = v * cos_theta * cos_alpha
    y_dot         = v * sin_theta * cos_alpha
    theta_dot     = v/dist_d * sin_alpha
    v_dot         = -1.0/tau_v * v     + 1.0/tau_v * v_ref
    alpha_dot     = -1.0/tau_a * alpha + 1.0/tau_a * alpha_ref
    v_ref_dot     = dv_ref
    alpha_ref_dot = dalpha_ref

    f_expl = ca.vertcat(x_dot, y_dot, theta_dot,
                        v_dot, alpha_dot,
                        v_ref_dot, alpha_ref_dot)
    f_impl = sym_xdot - f_expl

    # model
    model = AcadosModel()
    model.name = model_name
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = sym_x
    model.xdot = sym_xdot
    model.u = sym_u
    model.p = sym_p

    # store meta information
    model.x_labels = ['$x$ [m]', '$y$ [m]', r'$\theta$ [rad]',
                      '$v$ [m/s]', r'$\alpha$ [rad]',
                      '$v_{ref}$ [m/s]', r'$\alpha_{ref}$ [rad]']
    model.u_labels = ['$dv_{ref}$ [m/s]', r'$d\alpha_{ref}$ [rad]']
    model.t_label = '$t$ [s]'

    return model