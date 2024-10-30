#!/usr/bin/env python3
"""
A node that uses a global constant COMMANDS that consists of 
pairs of speed_linear and speed_angular.
COMMANDS are published in Twist messages
(populated in the same way as in key_op)
on the topic robot_twist at intervals of one second.
When the end of the list is reached,
the node returns to the beginning of the list.
Homework 2 Problem 3 Question (node) 3.1
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

COMMANDS = (
    (1.0, 0.0),         # forward
    (1.0, 0.0),         # forward
    (0.0, 0.0),         # stop
    (0.0, 1.57079635),  # turn
    (0.0, 0.0),         # stop
)

class ScriptOp(Node):
    '''
    Inherited Node class that takes each pair of speeds from COMMANDS,
    and publishes them as Twist messages on topic robot_twist
    '''
    def __init__(self):
        super().__init__('script_op')

        # create publisher to topic robot_twist
        self.publisher = self.create_publisher(Twist,
                                                'robot_twist',
                                                10)

        # index to iterate through COMMANDS
        self.iter = 0

        # create 1 Hz timer for publishing
        self.timer = self.create_timer(1.0,
                                       self.timer_callback)

    def timer_callback(self):
        '''
        Populate Twist msg for publishing
        '''
        msg = Twist()
        msg.linear.x = COMMANDS[self.iter][0]
        msg.angular.z = COMMANDS[self.iter][1]
        self.publisher.publish(msg)
        self.iter = (self.iter + 1) % len(COMMANDS)


def main(args=None):
    '''
    Initialize and continue to spin node until user quits with 'q'
    '''
    rclpy.init(args=args)
    so = ScriptOp()
    rclpy.spin(so)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
