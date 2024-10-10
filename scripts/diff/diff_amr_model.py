from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos
from common import *

def export_diff_amr_model() -> AcadosModel:
    model_name = 'diff2amr'

    # states
    x = SX.sym('x')             # position x in map frame
    y = SX.sym('y')             # position y in map frame
    theta = SX.sym('theta')     # orientation theta in map frame
    vl = SX.sym('vl')           # linear velocity left wheel
    vr = SX.sym('vr')           # linear velocity right wheel
    vl_ref = SX.sym('vl_ref')   # reference linear velocity left wheel
    vr_ref = SX.sym('vr_ref')   # reference linear velocity right wheel
    sym_x = vertcat(x, y, theta,
                    vl, vr,
                    vl_ref, vr_ref)

    # controls
    dvl_ref = SX.sym('dv1_ref') # variation of the reference linear velocity left wheel
    dvr_ref = SX.sym('dv2_ref') # variation of the reference linear velocity right wheel
    sym_u = vertcat(dvl_ref, dvr_ref)

    # xdot for f_impl
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    theta_dot = SX.sym('theta_dot')
    vl_dot = SX.sym('vl_dot')
    vr_dot = SX.sym('vr_dot')
    vl_ref_dot = SX.sym('vl_ref_dot')
    vr_ref_dot = SX.sym('vr_ref_dot')
    sym_xdot = vertcat(x_dot, y_dot, theta_dot,
                       vl_dot, vr_dot,
                       vl_ref_dot, vr_ref_dot)

    # dynamics
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    v  = ( vr + vl )/2.0
    w  = ( vr - vl )/B

    # dynamics
    x_dot      = v * cos_theta
    y_dot      = v * sin_theta
    theta_dot  = w
    vl_dot     = -1.0/TAU_V * vl + 1.0/TAU_V * vl_ref
    vr_dot     = -1.0/TAU_V * vr + 1.0/TAU_V * vr_ref
    vl_ref_dot = dvl_ref
    vr_ref_dot = dvr_ref

    f_expl = vertcat(x_dot, y_dot, theta_dot, 
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

    # store meta information
    model.x_labels = ['$x$ [m]', '$y$ [m]', r'$\theta$ [rad]', 
                      '$vl$ [m/s]', '$vr$ [m/s]',
                      '$vl_{ref}$ [m/s]', '$vr_{ref}$ [m/s]']
    model.u_labels = ['$dvl_{ref}$ [m/s]', '$dvr_{ref}$ [m/s]']
    model.t_label = '$t$ [s]'

    return model