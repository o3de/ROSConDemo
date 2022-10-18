#!/usr/bin/env python3

# Copyright 2019-2022 Robotec.ai.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

class JoyToAckermann(Node):

    def __init__(self):
        super().__init__('joy_to_ackermann')
        self.publisher = self.create_publisher(AckermannDrive, 'ackermann_vel', 10)
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10) 

    def joy_callback(self, msg):
        drive = AckermannDrive()
        drive.speed = 1.5*msg.axes[1]
        drive.steering_angle = 1.1*msg.axes[2]
        self.publisher .publish(drive)
    


def main(args=None):
    rclpy.init(args=args)

    joyToAckermann = JoyToAckermann()

    rclpy.spin(joyToAckermann)

    joyToAckermann.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
