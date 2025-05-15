from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

class Diff2AmrModel:
    def __init__(self, tau_v, l1_plus_l2):
        # states
        x = SX.sym('x')             # position x in map frame
        y = SX.sym('y')             # position y in map frame
        theta = SX.sym('theta')     # orientation theta in map frame
        self.sym_x = vertcat(x, y, theta)

        # controls
        vr = SX.sym('vr') # right wheel velocity
        vl = SX.sym('vl') # left wheel velocity
        self.sym_u = vertcat(vr, vl)

        # parameters
        dist_b = SX.sym('dist_b') # distance between the wheels [m]
        self.sym_p = dist_b

        # xdot for f_impl
        x_dot = SX.sym('x_dot')
        y_dot = SX.sym('y_dot')
        theta_dot = SX.sym('theta_dot')
        self.sym_xdot = vertcat(x_dot, y_dot, theta_dot)

        # dynamics
        cos_theta = cos(theta)
        sin_theta = sin(theta)
        v  = ( vr + vl )/2.0
        w  = ( vr - vl )/dist_b

        x_dot      = v * cos_theta
        y_dot      = v * sin_theta
        theta_dot  = w

        self.f_expl = vertcat(x_dot, y_dot, theta_dot)
        self.f_impl = self.sym_xdot - self.f_expl

    def export_acados_model(self) -> AcadosModel:
        model = AcadosModel()
        model.name = 'diff2amr'
        model.x = self.sym_x
        model.u = self.sym_u
        model.p = self.sym_p
        model.xdot = self.sym_xdot
        model.f_impl_expr = self.f_impl
        model.f_expl_expr = self.f_expl

        # store meta information
        model.x_labels = ['$x$ [m]', '$y$ [m]', r'$\theta$ [rad]']
        model.u_labels = ['$vl$ [m/s]', '$vr$ [m/s]']
        model.t_label = '$t$ [s]'

        return model