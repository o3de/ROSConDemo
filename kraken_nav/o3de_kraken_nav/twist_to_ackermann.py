#!/usr/bin/env python3

import math
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

class TwistToAckermann(Node):
  def __init__(self):
    super().__init__('twist_to_ackermann')
    self.wheelbase_ = 2.0
    self.sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
    self.pub_ = self.create_publisher(AckermannDrive, 'ackermann_vel', 10)

  def twist_to_ackermann(self, vel, omega):
    if omega == 0 or vel == 0:
      return 0

    radius = vel/ omega
    steering = math.atan(self.wheelbase_ / radius)
    return float(steering)

  def cmd_vel_cb(self, data):
    vel = data.linear.x
    steering = self.twist_to_ackermann(vel, data.angular.z)
    # print(f"v: {vel} steering: {steering}")
    
    msg = AckermannDrive()
    msg.steering_angle = float(steering)
    msg.speed = vel
    
    self.pub_.publish(msg)

def main(args=None): 
  rclpy.init(args=args)
  tta = TwistToAckermann()
  rclpy.spin(tta)
  
  tta.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()
