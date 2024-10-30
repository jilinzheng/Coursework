#!/usr/bin/env python3
"""
Simple demo that listens to std_msgs/Strings published
to the 'chatter' topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    '''
    A simple Node class that logs messages on /chatter
    '''
    def __init__(self):
        super().__init__('listener')
        self.subscriber = self.create_subscription(String, 'chatter',
                                                   self.listener_callback, 10)

    def listener_callback(self, msg):
        """
        Callback to receive a message
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    minimal_subscriber = Listener()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly,
    # otherwise it will be done automatically
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
