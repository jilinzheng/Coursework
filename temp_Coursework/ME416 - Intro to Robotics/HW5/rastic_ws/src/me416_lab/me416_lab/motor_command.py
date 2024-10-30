#!/usr/bin/env python3
"""
Homework 1 Question 2.4-6
"""

import me416_utilities
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from me416_msgs.msg import MotorSpeedsStamped
import robot_model

motor_left = me416_utilities.MotorSpeedLeft(0.9)
motor_right = me416_utilities.MotorSpeedRight()


class MotorCommand(Node):
    ''' Inherited Node class that subscribes and publishes concatenated messages with a delay '''
    def __init__(self):
        ''' Setup subscriber to robot_twist and publisher to motor_speeds'''
        super().__init__('motor_command')
        self.subscriber = self.create_subscription(Twist,
                                                   'robot_twist',
                                                   self.subscriber_callback,
                                                   10)
        self.publisher = self.create_publisher(MotorSpeedsStamped,
                                               'motor_speeds',
                                               10)
        self.msg = MotorSpeedsStamped()


    def subscriber_callback(self, twist_msg):
        ''' Callback for subscriber '''
        # Question 2.5
        left_speed, right_speed = robot_model.twist_to_speeds(twist_msg.linear.x,
                                                              twist_msg.angular.z)
        motor_left.set_speed(left_speed)
        motor_right.set_speed(right_speed)

        # Question 2.6
        msg = MotorSpeedsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.left = left_speed
        msg.right = right_speed
        self.publisher.publish(msg)


def main(args=None):
    ''' Init ROS, launch node, spin, cleanup '''
    rclpy.init(args=args)
    motor_command = MotorCommand()
    rclpy.spin(motor_command)

    # Node cleanup
    motor_command.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
