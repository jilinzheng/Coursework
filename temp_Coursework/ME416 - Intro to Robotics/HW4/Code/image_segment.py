#!/usr/bin/env python3
"""
Node to perform real-time extraction of centroid
of line from iamges acquired by robot
"""


import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import image_processing as ip


class ImageSegment(Node):
    """
    A publish-upon-message node for flipping images in a stream
    """
    def __init__(self):
        super().__init__("image_publisher")
        self.segment_publisher = self.create_publisher(Image, "/image/segmented", 1)
        self.centroid_publisher = self.create_publisher(PointStamped, "/image/centroid", 1)
        self.create_subscription(Image, "/image_raw", self.camera_callback, 1)
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        """
        Get image from topic, convert to HSV, perform operations, and republish
        """
        img = self.bridge.imgmsg_to_cv2(msg)
        cv2.cvtColor(src=img, dst=img, code=cv2.COLOR_BGR2HSV)
        low, high = ip.classifier_parameters()

        # reuse img in the following operations because the original is not needed
        img = ip.image_segment(img, low, high)
        centroid = ip.image_centroid_horizontal(img)
        img = ip.image_one_to_three_channels(img)
        img = ip.image_line_vertical(img, centroid)

        centroid_msg = PointStamped()
        centroid_msg.point.x = float(centroid)
        centroid_msg.header.stamp = self.get_clock().now().to_msg()

        self.segment_publisher.publish(self.bridge.cv2_to_imgmsg(img, encoding="rgb8"))
        self.centroid_publisher.publish(centroid_msg)


def main(args=None):
    """
    ImageSegment node
    """
    rclpy.init(args=args)
    image_publisher = ImageSegment()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
