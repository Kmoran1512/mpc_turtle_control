# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import matplotlib as plt
from casadi import *
from casadi.tools import *
import pdb
import sys, os

import numpy as np
import do_mpc

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


from .mpc_model import mpc_model
from .mpc_controller import mpc_controller
from .mpc_simulator import mpc_simulator


class MpcNode(Node):
    MAX_ANG_VEL = 1
    MAX_LIN_VEL = 1
    SPEED_FACTOR = 1
    REFRESH_RATE = 1

    def __init__(self, target_x, target_y):
        super().__init__("minimal_publisher")

        self.iteration = 0
        self.dt = 1
        self.init = [0, 0, 0]
        self.target = [target_x, target_y, 0]
        self.input_constraints = [self.MAX_LIN_VEL, self.MAX_ANG_VEL]

        self.isPositionSet = False

        self.x = 0
        self.y = 0
        self.theta = 0

        self.subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.setPose, 10
        )
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.model = mpc_model(self.dt, self.target)
        self.mpc = mpc_controller(self.model, self.dt, self.input_constraints)
        self.simulator = mpc_simulator(self.model, self.dt)

        self.timer = self.create_timer(self.REFRESH_RATE, self.runMPC)

    def runMPC(self):
        if not self.isPositionSet:
            return

        self.iteration += 1
        x0 = self.simulator.x0
        x0["x_pos"] = self.x
        x0["y_pos"] = self.y
        x0["theta_pos"] = self.theta

        self.mpc.x0 = x0
        self.mpc.set_initial_guess()
        u0 = self.mpc.make_step(x0)

        pub_msg = Twist()
        pub_msg.linear.x = u0[0][0] * self.SPEED_FACTOR
        pub_msg.angular.z = u0[1][0] * self.SPEED_FACTOR
        self.publisher_.publish(pub_msg)

    def setPose(self, poseMsg: Pose):
        self.x = poseMsg.x
        self.y = poseMsg.y
        self.theta = poseMsg.theta

        self.isPositionSet = True


def main(args=None):
    goal_x = float(input("goal x: "))
    goal_y = float(input("goal y: "))

    rclpy.init(args=args)

    controller = MpcNode(goal_x, goal_y)
    print("ready for take off")

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
