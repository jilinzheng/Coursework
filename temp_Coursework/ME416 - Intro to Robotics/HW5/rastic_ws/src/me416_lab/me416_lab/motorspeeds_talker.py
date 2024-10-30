#!/usr/bin/env python3
"""
Simple demo of how to publish me416_msgs/MotorSpeedsStamped messages
"""

import random

import rclpy
from me416_msgs.msg import MotorSpeedsStamped
from rclpy.node import Node


class MotorSpeedsTalker(Node):
    '''
    Simple Node class that uses a timer to publish on /motor_speeds
    '''
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(MotorSpeedsStamped, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        '''
        Method called by the Timer object
        '''
        msg = MotorSpeedsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'random'
        msg.left = random.random()
        msg.right = random.random()
        self.publisher.publish(msg)


def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)

    minimal_publisher = MotorSpeedsTalker()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly, otherwise
    # it will be done automatically
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
