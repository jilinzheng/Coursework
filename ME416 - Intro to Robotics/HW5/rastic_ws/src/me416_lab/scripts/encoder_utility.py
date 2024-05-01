#!/usr/bin/env python3

import argparse
import time

from periphery import GPIO

pin_id = [(1, 88), (1, 87), (0, 5), (0, 4)]
pins = [GPIO(f'/dev/gpiochip{chip}', line, 'in') for chip, line in pin_id]


def decode_decimal(pair):
    """
    Given a pair of zeros/ones, interpret as binary
    and return the decimal value
    """
    return pair[1] + 2 * pair[0]


def decode_gray(pair):
    """
    Given a pair of zeros/ones, interpret as gray code
    and return the decimal value
    """
    gray = [0, 1, 3, 2]
    return gray[decode_decimal(pair)]


def decode_difference(pair, pair_prev):
    """
    Given consecutive pairs of gray codes,
    and compute the difference (+1,-1)
    """
    diff = (decode_gray(pair) - decode_gray(pair_prev)) % 4
    if diff == 3:
        diff = -1
    return diff


def main(args):
    """
    Continuously check the GPIO pins, and print when different
    """
    status_prev = None
    counts = [0, 0]
    while True:
        status = [int(pin.read()) for pin in pins]
        time.sleep(1e-5)
        if status != status_prev:
            if args.mode == 'binary':
                pos = status
            elif args.mode == 'decimal':
                pos = [
                    decode_decimal(status[0:2]),
                    decode_decimal(status[2:4])
                ]
            elif args.mode == 'gray':
                pos = [decode_gray(status[0:2]), decode_gray(status[2:4])]
            elif args.mode == 'differential':
                if status_prev is not None:
                    pos = [
                        decode_difference(status[0:2], status_prev[0:2]),
                        decode_difference(status[2:4], status_prev[2:4])
                    ]
                else:
                    pos = None
            else:  # integral
                if status_prev is not None:
                    counts = [
                        counts[0] +
                        decode_difference(status[0:2], status_prev[0:2]),
                        counts[1] +
                        decode_difference(status[2:4], status_prev[2:4])
                    ]
                pos = counts
            print(pos)
            status_prev = status


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Utility to test ROSBot encoders.',
        epilog="example usage: ./encoder_utility.py --mode differential ",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--mode',
        type=str,
        default='binary',
        help=
        'How to present information: "binary", "decimal", "gray", "differential", or "integral"'
    )
    args = parser.parse_args()
    main(args)
