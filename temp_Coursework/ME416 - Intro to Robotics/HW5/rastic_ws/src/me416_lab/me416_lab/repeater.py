#!/usr/bin/env python3
"""
Simple repeater demo that receives std_msgs/Strings messages from the 'chatter'
 topic, modifies them, and then sends them on the 'chatter_repeated' topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Repeater(Node):
    '''
    Simple Node class that uses a timer to publish on /chatter
    '''
    def __init__(self):
        '''
        Setup subscriber and publisher
        '''
        super().__init__('repeater')
        self.subscriber = self.create_subscription(String, 'chatter',
                                                   self.msg_callback, 10)
        self.publisher = self.create_publisher(String, 'chatter_repeated', 10)

    def msg_callback(self, msg):
        """
        Callback to receive a message and repeat it with some modifications
        """

        # Take content from the message
        content = msg.data
        # Manipulate content (change some of the character to '*')
        content = ''.join(['*' if x in 'aeiou01234' else x for x in content])

        # Publish a new message with the modified string
        msg_repeated = String()
        msg_repeated.data = content
        self.publisher.publish(msg_repeated)

        # Show some logging information (optional)
        self.get_logger().info(f'I heard: "{msg.data}"')
        self.get_logger().info(f'I repeated: "{msg_repeated.data}"')


def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    repeater = Repeater()
    rclpy.spin(repeater)

    # node cleanup
    repeater.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
