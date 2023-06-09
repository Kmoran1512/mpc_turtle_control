from casadi import *
from casadi.tools import *
import do_mpc


def mpc_controller(model: do_mpc.model.Model, dt, input_constraints):
    mpc = do_mpc.controller.MPC(model)

    mpc.settings.n_horizon = 20
    mpc.settings.t_step = dt

    lterm = model.aux["lagrange_term"]
    mterm = model.aux["meyer_term"]

    mpc.set_objective(lterm=lterm, mterm=mterm)
    mpc.set_rterm(linear_velocity=0, angular_velocity=0)

    max_linear_velocity = input_constraints[0]
    max_angular_velocity = input_constraints[1]

    mpc.bounds["lower", "_u", "linear_velocity"] = -max_linear_velocity
    mpc.bounds["lower", "_u", "angular_velocity"] = -max_angular_velocity
    mpc.bounds["upper", "_u", "linear_velocity"] = max_linear_velocity
    mpc.bounds["upper", "_u", "angular_velocity"] = max_angular_velocity

    mpc.setup()

    return mpc
