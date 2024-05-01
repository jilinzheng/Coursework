#!/usr/bin/env/python3
"""
Publishes odometry
"""

import numpy as np
import rclpy
import robot_model as rm
from geometry_msgs.msg import PoseStamped
from me416_msgs.msg import MotorSpeedsStamped
from rclpy.node import Node
from transforms3d.euler import euler2quat


class Odometry(Node):
    '''
    Publishes a pose using Euler integration from motor speeds
    '''
    def __init__(self):
        super().__init__('odometry_encoders')
        self.publisher = None  # create publisher for /odom_euler
        # create subscriber for /encoders with self.encoders_callback
        # create a timer that calls self.timer_callback at 0.5s intervals
        # initialize state estimate self.z_state
        # create object for StampedMsgRegister

    def timer_callback(self):
        '''
        Publish current pose estimate
        '''
        msg = PoseStamped()
        # Fill orientation
        theta = 0  # extract theta from the current state
        quat = euler2quat(0, 0, theta)
        msg.pose.orientation.x = quat[1]
        msg.pose.orientation.y = quat[2]
        msg.pose.orientation.z = quat[3]
        msg.pose.orientation.w = quat[0]
        # Fill position
        # set msg.pose.position using the current state
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # publish the message

    def encoders_callback(self, msg):
        '''
        Integrate the motor speeds in pose estimate using Euler's method
        '''
        # obtain elapsed time from previous call using the StampedMsgRegister attribute
        # Note: the elapsed time cannot will not be available on the first call
        # obtain input u for the model from msg
        # apply Euler step to update current state estimate


def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
