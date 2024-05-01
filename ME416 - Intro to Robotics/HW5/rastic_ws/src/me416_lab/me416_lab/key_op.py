#!/usr/bin/env python3
"""
A node that collects key presses from the user,
passes them to a KeysToVelocies object,
and publishes the velocities as Twist messages.
Homework 2 Problem 2 Question (report) 2.1
"""

import me416_utilities as mu
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_model import KeysToVelocities


class KeyOp(Node):
    '''
    Inherited Node class that collects key presses from user,
    passes them to a KeysToVelocities object,
    and publishes the velocities as Twist messages.
    '''
    def __init__(self):
        super().__init__('key_op')

        # create publisher to topic robot_twist
        self.publisher = self.create_publisher(Twist,
                                               'robot_twist',
                                               10)

        # create object from class KeysToVelocities
        self.key_translate = KeysToVelocities()

        print(f'''
Available commands:
w, W: Increase linear speed
s, S: Decrease linear speed
a, A: Increase angular speed
d, D: Decrease angular speed
z, Z: Set linear speed to zero
c, C: Set angular speed to zero
x, X: Set both linear and angular speed to zero
q, Q: Quit key translation to velocities

Current speeds:
speed_linear = {self.key_translate.speed_linear}
speed_angular = {self.key_translate.speed_angular}
              ''')

        # key presses checked at 50 Hz
        self.timer = self.create_timer(1.0/50,
                                       self.timer_callback)
        self.getch = mu._Getch()
        self.is_running = True

    def timer_callback(self):
        '''
        Process key pressed and publish Twist msg
        '''
        key = self.getch()
        if key.isalpha():
            [speed_linear, speed_angular, text_description] = self.key_translate.update_speeds(key)
            print(f'''
{text_description}
Updated speeds:
speed_linear = {speed_linear}
speed_angular = {speed_angular}
                  ''')

            msg = Twist()
            msg.linear.x = self.key_translate.speed_linear
            msg.angular.z = self.key_translate.speed_angular
            self.publisher.publish(msg)

            if key in ('q', 'Q'):
                self.get_logger().info(f'Shutdown initiated by {self.get_name()}')
                self.destroy_node()
                self.is_running = False


def main(args=None):
    '''
    Initialize and continue to spin node until user quits with 'q'
    '''
    rclpy.init(args=args)
    ko = KeyOp()
    while ko.is_running:
        rclpy.spin_once(ko)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
