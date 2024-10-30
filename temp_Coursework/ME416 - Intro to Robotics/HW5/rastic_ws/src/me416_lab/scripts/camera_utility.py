#!/usr/bin/env python3
"""Utility to test ROSBot camera."""
import argparse
from datetime import datetime

import cv2


def capture_n(cap, n_frames=30):
    """
    Capture n images, and save them in different files with the current date
    """
    filename_prefix = 'camera_utility_' + datetime.today().strftime(
        '%Y_%m_%d_%H_%M_%S') + '_'
    for count in range(n_frames):
        ret, frame = cap.read()
        if ret is True:
            filename = f'{filename_prefix}{count:03}.png'
            cv2.imwrite(filename, frame)
            print(filename)
    return frame


def main(args):
    """
    Sets up camera via OpenCV, then captures n pictures
    """

    cap = cv2.VideoCapture(1)
    # Set image width to 640 (will change height automatically)
    cap.set(3, 640)
    capture_n(cap, args.n)
    cap.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Utility to test ROSBot camera.',
        epilog="example usage: ./camera_utility.py -n 30 ",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-n',
                        type=int,
                        default=1,
                        help='How many snapshots to capture"')
    args = parser.parse_args()
    main(args)
