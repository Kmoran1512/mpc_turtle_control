from casadi import *
from casadi.tools import *
import do_mpc


def mpc_simulator(model: do_mpc.model.Model, dt):
    simulator = do_mpc.simulator.Simulator(model)

    simulator.settings.t_step = dt

    simulator.setup()

    return simulator
