#!/usr/bin/env/python3
"""
Publishes odometry
"""

#from matplotlib.pyplot import step
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from transforms3d.euler import euler2quat
from me416_msgs.msg import MotorSpeedsStamped
import robot_model as rm


class Odometry(Node):
    '''
    Publishes a pose using Euler integration from motor speeds
    '''
    def __init__(self):
        super().__init__('odometry_encoders')
        # create publisher for /odom_euler
        self.publisher = self.create_publisher(PoseStamped,
                                               'odom_euler',
                                               10)
        # create subscriber for /encoders with self.encoders_callback
        self.subscriber = self.create_subscription(MotorSpeedsStamped,
                                                   'encoders',
                                                   self.encoders_callback,
                                                   10)
        # create a timer that calls self.timer_callback at 0.5s intervals
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,
                                       self.timer_callback)
        # initialize state estimate self.z_state
        self.z_state = np.array([[0.], [0.], [0.]])
        # create object for StampedMsgRegister
        self.stamped_msg_reg = rm.StampedMsgRegister()

    def timer_callback(self):
        '''
        Publish current pose estimate
        '''
        msg = PoseStamped()
        # Fill orientation
        theta = self.z_state[2, 0]  # extract theta from the current state
        quat = euler2quat(0, 0, theta)
        msg.pose.orientation.x = quat[1]
        msg.pose.orientation.y = quat[2]
        msg.pose.orientation.z = quat[3]
        msg.pose.orientation.w = quat[0]
        # Fill position
        msg.pose.position.x = self.z_state[0, 0]
        msg.pose.position.y = self.z_state[1, 0]
        msg.pose.position.z = 0.
        # set msg.pose.position using the current state
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # publish the message
        self.publisher.publish(msg)

    def encoders_callback(self, msg):
        '''
        Integrate the motor speeds in pose estimate using Euler's method
        '''
        # obtain elapsed time from previous call using the StampedMsgRegister attribute
        # Note: the elapsed time cannot will not be available on the first call
        step_size = self.stamped_msg_reg.replace_and_compute_delay(msg)[0]
        # obtain input u for the model from msg
        u_scale = 3600.
        u = np.array([[msg.left], [msg.right]]) / u_scale
        # apply Euler step to update current state estimate
        self.z_state = rm.euler_step(self.z_state, u, step_size)
        # explicitly call the timer_callback to reduce lag in odometry update
        self.timer_callback()
        #OPTIONAL CODE, BUT NEVER EXECUTED BECAUSE STEP_SIZE WAS ZERO OR TOO LARGE
        #if step_size != 0 and step_size <= 0.02:
        #    self.z_state = rm.euler_step(self.z_state, u, step_size)
            # explicitly call the timer_callback to reduce lag in odometry update
        #    self.timer_callback()


def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
