#!/usr/bin/env python3
""" Node that applies the PIDController from control.py to the topic /error_signal
"""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64

import controller


class ControllerTest(Node):
    def __init__(self):
        super().__init__('controller_test_node')
        # publish PID topics based on error signal stamped
        self.create_subscription(PointStamped, '/error_signal_stamped',
                                 self.error_callback, 10)
        self.pub = {}
        for pub_name in ['proportional', 'derivative', 'integral']:
            self.pub[pub_name] = self.create_publisher(Float64,
                                                       '/control_' + pub_name,
                                                       2)
        self.stamp_previous = None
        self.pid = controller.PID()

    def error_callback(self, msg):
        '''
        Get error, compute control
        '''
        error_signal = msg.point.x
        stamp = msg.header.stamp

        control_proportional = self.pid.proportional(error_signal)
        msg_proportional = Float64()
        msg_proportional.data = control_proportional
        self.pub['proportional'].publish(msg_proportional)
        if self.stamp_previous:

            t_now = Time.from_msg(stamp)
            t_prev = Time.from_msg(self.stamp_previous)

            time_delay = (t_now.nanoseconds - t_prev.nanoseconds) / 1000000000.

            control_derivative = self.pid.derivative(error_signal, time_delay)
            msg_derivative = Float64()
            msg_derivative.data = control_derivative
            self.pub['derivative'].publish(msg_derivative)

            control_integral = self.pid.integral(error_signal, time_delay)
            msg_integral = Float64()
            msg_integral.data = control_integral
            self.pub['integral'].publish(msg_integral)

        self.stamp_previous = stamp


def main(args=None):
    """Node setup and use"""
    # Initialize node
    rclpy.init(args=args)
    ct = ControllerTest()
    rclpy.spin(ct)
    ct.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
