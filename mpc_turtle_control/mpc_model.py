import do_mpc
from casadi import *
from casadi.tools import cos, sin


def mpc_model(dt, target):
    model_type = "discrete"
    model = do_mpc.model.Model(model_type)

    x_pos = model.set_variable("_x", "x_pos")
    y_pos = model.set_variable("_x", "y_pos")
    theta_pos = model.set_variable("_x", "theta_pos")

    linear_velocity = model.set_variable("_u", "linear_velocity")
    angular_velocity = model.set_variable("_u", "angular_velocity")

    target_x = target[0]
    target_y = target[1]
    target_theta = target[2]

    # Can you explain these objective functions?
    model.set_expression(
        expr_name="lagrange_term", expr=(linear_velocity**2 + angular_velocity**2)
    )
    model.set_expression(
        expr_name="meyer_term",
        expr=(
            (target_x - x_pos) ** 2
            + (target_y - y_pos) ** 2
            + (target_theta - theta_pos) ** 2
        ),
    )

    # These make sense to me
    model.set_rhs("x_pos", x_pos + linear_velocity * cos(theta_pos) * dt)
    model.set_rhs("y_pos", y_pos + linear_velocity * sin(theta_pos) * dt)
    model.set_rhs("theta_pos", theta_pos + angular_velocity * dt)

    model.setup()

    return model
