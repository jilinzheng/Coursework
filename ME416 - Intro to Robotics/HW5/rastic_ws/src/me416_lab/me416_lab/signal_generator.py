#!/usr/bin/env python3
""" Generate a time-varying error signal on the topics:
 - /error_signal_stamped, which is of type PointStamped (error is in the point.x field)
 - /error_signal, which is of type Float64
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64

SIGNAL_FREQUENCY = 1


class SignalGenerator(Node):
    '''
    A class to generate signals
    '''
    def __init__(self):
        ''' Node setup '''
        super().__init__('signal_generator')
        # create publishers
        self.pub_stamped = self.create_publisher(PointStamped,
                                                 'error_signal_stamped', 2)
        self.pub = self.create_publisher(Float64, 'error_signal', 2)

        # set up the timer to handle publishing
        rate = 10  # Hz
        self.timer = self.create_timer(1 / rate, self.timer_callback)

    def timer_callback(self):
        '''Generate and publish signals'''
        msg = PointStamped()

        # Get current time and the value of a cosine wave
        current_time = self.get_clock().now()
        float_time = current_time.nanoseconds / 1000000000.

        error_signal = math.cos(2 * math.pi * SIGNAL_FREQUENCY * float_time)

        # Publish on the stamped topic
        msg.header.stamp = current_time.to_msg()
        msg.point.x = error_signal
        self.pub_stamped.publish(msg)

        # Publish on the non-stamped topic
        msg_float = Float64()
        msg_float.data = error_signal

        self.pub.publish(msg_float)


def main(args=None):
    '''Initialize ROS, launch node, spin, cleanup'''

    rclpy.init(args=args)
    sig_gen = SignalGenerator()
    rclpy.spin(sig_gen)
    sig_gen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
