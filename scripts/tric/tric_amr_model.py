from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos
from common import *

def export_tric_amr_model() -> AcadosModel:
    model_name = 'tric3amr'

    # states
    x = SX.sym('x')                   # position x in map frame
    y = SX.sym('y')                   # position y in map frame
    theta = SX.sym('theta')           # orientation theta in map frame
    v = SX.sym('v')                   # linear velocity
    alpha = SX.sym('alpha')           # steering angle
    v_ref = SX.sym('v_ref')           # reference linear velocity
    alpha_ref = SX.sym('alpha_ref')   # reference steering angle
    sym_x = vertcat(x, y, theta,
                    v, alpha,
                    v_ref, alpha_ref)

    # controls
    dv_ref = SX.sym('dv_ref')         # variation of the reference linear velocity
    dalpha_ref = SX.sym('dalpha_ref') # variation of the reference steering angle
    sym_u = vertcat(dv_ref, dalpha_ref)

    # xdot for f_impl
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    theta_dot = SX.sym('theta_dot')
    v_dot = SX.sym('v_dot')
    alpha_dot = SX.sym('alpha_dot')
    v_ref_dot = SX.sym('v_ref_dot')
    alpha_ref_dot = SX.sym('alpha_ref_dot')
    sym_xdot = vertcat(x_dot, y_dot, theta_dot,
                       v_dot, alpha_dot,
                       v_ref_dot, alpha_ref_dot)

    # dynamics
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    cos_alpha = sin(alpha)
    sin_alpha = sin(alpha)

    # dynamics
    x_dot         = v * cos_theta * cos_alpha
    y_dot         = v * sin_theta * cos_alpha
    theta_dot     = v/DIST_D * sin_alpha
    v_dot         = -1.0/TAU_V * v     + 1.0/TAU_V * v_ref
    alpha_dot     = -1.0/TAU_A * alpha + 1.0/TAU_A * alpha_ref
    v_ref_dot     = dv_ref
    alpha_ref_dot = dalpha_ref

    f_expl = vertcat(x_dot, y_dot, theta_dot, 
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

    # store meta information
    model.x_labels = ['$x$ [m]', '$y$ [m]', r'$\theta$ [rad]', 
                      '$v$ [m/s]', r'$\alpha$ [rad]',
                      '$v_{ref}$ [m/s]', r'$\alpha_{ref}$ [rad]']
    model.u_labels = ['$dv_{ref}$ [m/s]', r'$d\alpha_{ref}$ [rad]']
    model.t_label = '$t$ [s]'

    return model