#!/usr/bin/env python3

#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive


class JoyToAckermann(Node):
    """ROS2 Node to convert joystick message to Ackermann Drive message."""

    def __init__(self):
        super().__init__('joy_to_ackermann')
        self.publisher = self.create_publisher(AckermannDrive, 'ackermann_vel', 10)
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

    def joy_callback(self, msg):
        """Create Ackermann message from the joystick message."""
        drive = AckermannDrive()
        drive.speed = 1.5 * msg.axes[1]
        drive.steering_angle = 1.1 * msg.axes[0]
        self.publisher.publish(drive)


def main(args=None):
    rclpy.init(args=args)

    joyToAckermann = JoyToAckermann()

    rclpy.spin(joyToAckermann)

    joyToAckermann.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
