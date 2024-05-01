#!/usr/bin/env python3
"""
Simple node that subscribes to images from a topic, flips them,
and publishes on another
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisher(Node):
    '''
    A publish-upon-message node for flipping images in a stream
    '''
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'image_flip', 1)
        self.create_subscription(Image, '/image_raw', self.camera_callback, 1)
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        '''
        Get  images from topic, flip, and republish
        '''
        img = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.flip(
            img,  # image to flip
            0  # means vertical flip
        )
        self.publisher.publish(self.bridge.cv2_to_imgmsg(img, encoding='rgb8'))


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
