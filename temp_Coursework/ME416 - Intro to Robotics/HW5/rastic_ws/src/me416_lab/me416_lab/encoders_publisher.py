#!/usr/bin/env python3
"""
Publishes the values of the encoders in counts per seconds (CPS).
To convert to rotations per minute (RPM) of the output shaft
RPM = CPS*(60 seconds/minute) / (GEAR RATIO output:input) / (COUNTS Per Rev)
Example (12 CPR encoder with 120:1 gear ratio, running at 3600 CPS):
3600 CPS * 60 / 120 / 12 = 150RPM (which is the spec'ed speed for our current
motors)
"""

from platform import machine

import rclpy
from me416_msgs.msg import MotorSpeedsStamped
from rclpy.node import Node

if machine() != 'x86_64':
    # Import library for GPIO only if on the Libre board
    from periphery import GPIO
    PIN_ID = [
        (0, 5),
        (0, 4),  # Left
        (1, 88),
        (1, 87)  # Right
    ]

POLLING_METHOD = 'timer'

if POLLING_METHOD == 'sleep':
    from time import sleep


class Encoders():
    '''
    Class to read from encoders (using polling), and estimate speed
    '''
    def __init__(self):
        if machine() != 'x86_64':
            self.pins = [
                GPIO(f'/dev/gpiochip{chip}', line, 'in')
                for chip, line in PIN_ID
            ]
        self.counts = [0, 0]
        self.status_prev = None

    def update(self):
        '''
        Read status from pin, and update counts
        '''
        if machine() == 'x86_64':
            # If the node is not running on the robot
            # give fake readings (rotation around the left wheel)
            status = [None, None]
            increments = [0, .3]
        else:
            # Obtain count increments from encoders,
            # use [0,0] if this is the first call
            status = [int(pin.read()) for pin in self.pins]
            if self.status_prev is None:
                increments = [0, 0]
            else:
                increments = [
                    self.decode_difference(status[0:2], self.status_prev[0:2]),
                    self.decode_difference(status[2:4], self.status_prev[2:4])
                ]

        # Update counts with increments
        self.counts = [
            self.counts[0] + increments[0], self.counts[1] + increments[1]
        ]
        # Save the status for next call
        self.status_prev = status

    def reset(self):
        """
        Set local counts to zero
        """
        self.counts = [0, 0]

    def decode_decimal(self, pair):
        """
        Given a pair of zeros/ones, interpret as binary
        and return the decimal value
        """
        return pair[1] + 2 * pair[0]

    def decode_gray(self, pair):
        """
        Given a pair of zeros/ones, interpret as gray code
        and return the decimal value
        """
        gray = [0, 1, 3, 2]
        return gray[self.decode_decimal(pair)]

    def decode_difference(self, pair, pair_prev):
        """
        Given consecutive pairs of gray codes,
        and compute the difference (+1,-1)
        """
        diff = (self.decode_gray(pair) - self.decode_gray(pair_prev)) % 4
        if diff == 3:
            diff = -1
        return diff


class EncodersNode(Node):
    '''
    Node that reads from encoders and estimates velocity
    '''
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(MotorSpeedsStamped, '/encoders',
                                               10)
        # Timer for publishing
        self.create_timer(
            0.1,  # seconds
            self.publish)
        if POLLING_METHOD == 'timer':
            # Timer for polling encoders
            self.create_timer(
                1e-5,  # seconds
                self.polling)
        self.encoders = Encoders()
        self.velocity_estimate_previous_time = None
        self.clock = self.get_clock()

    def polling(self):
        '''
        Manually update encoder reading
        '''
        self.encoders.update()

    def velocity_estimate(self):
        '''
        Estimate velocity from encoder counts divided by elapsed time
        '''
        now = self.clock.now()
        counts = self.encoders.counts
        self.encoders.reset()

        if self.velocity_estimate_previous_time is None:
            velocity = [0., 0.]
        else:
            elapsed_ns = (now -
                          self.velocity_estimate_previous_time).nanoseconds
            velocity = [c * 1e9 / elapsed_ns for c in counts]
        self.velocity_estimate_previous_time = now
        return velocity

    def publish(self):
        '''
        Publish current velocity estimate
        '''
        msg = MotorSpeedsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'encoders'

        msg.left, msg.right = self.velocity_estimate()
        self.publisher.publish(msg)


def main(args=None):
    """Node setup and main ROS loop"""

    rclpy.init(args=args)
    node = EncodersNode()
    if POLLING_METHOD == 'timer':
        rclpy.spin(node)
    else:
        while True:
            node.polling()
            rclpy.spin_once(node)
            sleep(1e-5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    finally:
        # This is the place to put any "clean up" code that should be executed
        # on shutdown even in case of errors, e.g., closing files or windows
        pass
