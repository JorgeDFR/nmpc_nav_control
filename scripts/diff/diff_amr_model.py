import casadi as ca
from acados_template import AcadosModel

def export_diff_amr_model() -> AcadosModel:
    model_name = 'diff2amr'

    # states
    x = ca.SX.sym('x')             # position x in map frame
    y = ca.SX.sym('y')             # position y in map frame
    theta = ca.SX.sym('theta')     # orientation theta in map frame
    vl = ca.SX.sym('vl')           # linear velocity left wheel
    vr = ca.SX.sym('vr')           # linear velocity right wheel
    vl_ref = ca.SX.sym('vl_ref')   # reference linear velocity left wheel
    vr_ref = ca.SX.sym('vr_ref')   # reference linear velocity right wheel
    sym_x = ca.vertcat(x, y, theta,
                       vl, vr,
                       vl_ref, vr_ref)

    # controls
    dvl_ref = ca.SX.sym('dvl_ref') # variation of the reference linear velocity left wheel
    dvr_ref = ca.SX.sym('dvr_ref') # variation of the reference linear velocity right wheel
    sym_u = ca.vertcat(dvl_ref, dvr_ref)

    # parameters
    dist_b = ca.SX.sym('dist_b')
    tau_v = ca.SX.sym('tau_v')
    sym_p = ca.vertcat(dist_b, tau_v)

    # xdot for f_impl
    x_dot = ca.SX.sym('x_dot')
    y_dot = ca.SX.sym('y_dot')
    theta_dot = ca.SX.sym('theta_dot')
    vl_dot = ca.SX.sym('vl_dot')
    vr_dot = ca.SX.sym('vr_dot')
    vl_ref_dot = ca.SX.sym('vl_ref_dot')
    vr_ref_dot = ca.SX.sym('vr_ref_dot')
    sym_xdot = ca.vertcat(x_dot, y_dot, theta_dot,
                          vl_dot, vr_dot,
                          vl_ref_dot, vr_ref_dot)

    # dynamics
    cos_theta = ca.cos(theta)
    sin_theta = ca.sin(theta)
    v  = (vr + vl)/2.0
    w  = (vr - vl)/dist_b

    # dynamics
    x_dot      = v * cos_theta
    y_dot      = v * sin_theta
    theta_dot  = w
    vl_dot     = dvl_ref
    vr_dot     = dvr_ref
    vl_dot     = -1.0/tau_v * vl + 1.0/tau_v * vl_ref
    vr_dot     = -1.0/tau_v * vr + 1.0/tau_v * vr_ref
    vl_ref_dot = dvl_ref
    vr_ref_dot = dvr_ref

    f_expl = ca.vertcat(x_dot, y_dot, theta_dot,
                        vl_dot, vr_dot,
                        vl_ref_dot, vr_ref_dot)
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
                      '$vl$ [m/s]', '$vr$ [m/s]',
                      '$vl_{ref}$ [m/s]', '$vr_{ref}$ [m/s]']
    model.u_labels = ['$dvl_{ref}$ [m/s]', '$dvr_{ref}$ [m/s]']
    model.t_label = '$t$ [s]'

    return model