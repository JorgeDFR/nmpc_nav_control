from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_diff_amr_model(DIST_B) -> AcadosModel:
    model_name = 'diff2amr'

    # states
    x = SX.sym('x')             # position x in map frame
    y = SX.sym('y')             # position y in map frame
    theta = SX.sym('theta')     # orientation theta in map frame
    vl = SX.sym('vl')           # left wheel linear velocity 
    vr = SX.sym('vr')           # right wheel linear velocity 
    sym_x = vertcat(x, y, theta, vl, vr)

    # controls
    dvl = SX.sym('dvl')         # variation of the left wheel linear velocity 
    dvr = SX.sym('dvr')         # variation of the right wheel linear velocity
    sym_u = vertcat(dvl, dvr)

    # xdot for f_impl
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    theta_dot = SX.sym('theta_dot')
    vl_dot = SX.sym('vl_dot')
    vr_dot = SX.sym('vr_dot')
    sym_xdot = vertcat(x_dot, y_dot, theta_dot, vl_dot, vr_dot)

    # dynamics
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    v  = ( vr + vl )/2.0
    w  = ( vr - vl )/DIST_B

    # dynamics
    x_dot      = v * cos_theta
    y_dot      = v * sin_theta
    theta_dot  = w
    vl_dot     = dvl
    vr_dot     = dvr

    f_expl = vertcat(x_dot, y_dot, theta_dot, vl_dot, vr_dot)
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
                      '$vl$ [m/s]', '$vr$ [m/s]']
    model.u_labels = ['$dvl$ [m/s]', '$dvr$ [m/s]']
    model.t_label = '$t$ [s]'

    return model