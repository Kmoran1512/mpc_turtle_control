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

import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class MinimalPublisher(Node):
    MAX_VELOCITY = 1.0
    MAX_TURN = 1.0

    def __init__(self, goal):
        super().__init__("minimal_publisher")

        self.goal = goal

        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.set_location, 5
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_move)

    def set_location(self, pose_msg):
        self.location: Pose = pose_msg

    def publish_move(self):
        msg = self.predictiveControl(self.location)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    def predictiveControl(self, position: Pose):
        control = Twist()
        if position.x < self.goal:
            control.linear.x = self.MAX_VELOCITY

        return control


def main(args=None):
    goal = float(input("goal x"))

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher(goal)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
