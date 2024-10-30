#!/usr/bin/env python3
"""
Homework 5 Problem 1
"""


import robot_model, controller
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, Twist


class ControllerLine(Node):
    """ Simple PID controller that allows robot to follow line while moving forward. """
    def __init__(self):
        """ Initialize node per assignment instructions, specifics commented below. """
        super().__init__('controller_line')
        # 1. Initialize attributes
        self.lin_speed = 0.055
        self.gain_kp = 0.004
        self.gain_kd = 0.003
        self.gain_ki = 0.003
        # 2. Initialize subscriber and two publishers
        self.subscriber = self.create_subscription(PointStamped,
                                                   '/image/centroid',
                                                   self.subscriber_callback, 10)
        self.twist_publisher = self.create_publisher(Twist,
                                                     'robot_twist',
                                                     10)
        self.error_publisher = self.create_publisher(Float64,
                                                     'control_error',
                                                     10)
        # 3. Initialize controller.PID object
        self.pid = controller.PID(gain_kp=self.gain_kp,
                                  gain_kd=self.gain_kd,
                                  gain_ki=self.gain_ki)
        # 4. Initialize robot_model.StampedMsgRegister object
        self.stamped_msg_register = robot_model.StampedMsgRegister()

    def subscriber_callback(self, point_stamped_msg):
        """ Perform assignment specified operations, commented below. """
        # 1. Compute error_signal
        IMG_WIDTH = 320
        error_signal = Float64()
        error_signal.data = point_stamped_msg.point.x - (0.5 * IMG_WIDTH)
        # 2. Publish error_signal to topic control_error
        self.error_publisher.publish(error_signal)
        # 3. Obtain time_delay via self.stamped_msg_register
        time_delay = self.stamped_msg_register.replace_and_compute_delay(point_stamped_msg)[0]
        # 4. Initialize Twist msg
        msg = Twist()
        # 5. Set msg.linear.x
        msg.linear.x = self.lin_speed
        # 6. Set msg.angular.z
        msg.angular.z = self.pid.proportional(error_signal=error_signal.data) \
                        + self.pid.derivative(error_signal=error_signal.data, time_delay=time_delay) \
                        + self.pid.integral(error_signal=error_signal.data, time_delay=time_delay)
        # 7. Publish msg (to topic robot_twist)
        self.twist_publisher.publish(msg)


def main(args=None):
    ''' Init ROS, launch node, spin, cleanup '''
    rclpy.init(args=args)
    controller_line = ControllerLine()
    rclpy.spin(controller_line)

    # Node cleanup
    controller_line.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
