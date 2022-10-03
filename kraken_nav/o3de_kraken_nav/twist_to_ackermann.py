#!/usr/bin/env python3

import math
from rclpy.node import Node
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

class TwistToAckermann(Node):
  def __init__(self):
    super().__init__('twist_to_ackermann')

    self.declare_parameter('wheelbase', 2.0)
    
    self.declare_parameter('publish_zeros_on_timeout', True)
    self.declare_parameter('timeout_control_interval', 0.1)
    self.declare_parameter('control_timeout', 0.2)
    
    self.wheelbase_ = self.get_parameter('wheelbase').get_parameter_value().double_value
    self.timeout_control_interval = self.get_parameter('timeout_control_interval').get_parameter_value().double_value
    self.control_timeout = self.get_parameter('control_timeout').get_parameter_value().double_value
    self.publish_zeros_on_timeout =  self.get_parameter('publish_zeros_on_timeout').get_parameter_value().bool_value
    
    print(self.publish_zeros_on_timeout)

    self.last_message_time = self.get_clock().now()
    self.sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
    self.pub_ = self.create_publisher(AckermannDrive, 'ackermann_vel', 10)
    if self.publish_zeros_on_timeout:
      self.timer = self.create_timer(self.timeout_control_interval, self.control)

  def control(self):
    time_diff = (self.get_clock().now() - self.last_message_time)
    if time_diff.nanoseconds * 1e-9 > self.control_timeout:
      msg = AckermannDrive()
      msg.steering_angle = 0.0
      msg.speed = 0.0
      self.pub_.publish(msg)
      
  def twist_to_ackermann(self, vel, omega):
    if omega == 0 or vel == 0:
      return 0

    radius = vel/ omega
    steering = math.atan(self.wheelbase_ / radius)
    return float(steering)

  def cmd_vel_cb(self, data):
    self.last_message_time = self.get_clock().now()
    
    vel = data.linear.x
    steering = self.twist_to_ackermann(vel, data.angular.z)
    
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
