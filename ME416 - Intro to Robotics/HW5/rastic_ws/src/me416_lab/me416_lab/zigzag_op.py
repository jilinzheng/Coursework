#!/usr/bin/env python3
""" Node that outputs a sequence of forward-moving arches """

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ZigZagCommander(Node):
    """
    Publishes Twist commands on robot_twist topic corresponding to a
    sequence of forward-moving arches
    """
    def __init__(self):
        super().__init__('ZigZagCommander')
        self.pub = self.create_publisher(Twist, 'robot_twist', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.update_send)
        self.count = 0
        self.nb_actions = 2

    def update_send(self):
        """ Switch angular speed and publish a new Twist command """
        # Init message. It is a 3-D Twist, but we will use only two fields
        msg = Twist()

        # Fill in linear (x-axis) and angular (z-axis) velocities
        msg.linear.x = 0.5
        if self.count == 0:
            msg.angular.z = -0.5
        else:
            msg.angular.z = 0.5

        # Update count
        self.count = (self.count + 1) % self.nb_actions

        # Publish
        self.pub.publish(msg)


def main(args=None):
    '''
    Initialize and spin node
    '''
    rclpy.init(args=args)
    zzc = ZigZagCommander()
    rclpy.spin(zzc)
    zzc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
