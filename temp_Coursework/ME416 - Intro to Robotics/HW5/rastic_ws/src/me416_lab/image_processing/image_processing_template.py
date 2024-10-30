#!/usr/bin/env python3
"""
This is a library of functions for performing color-based image segmentation
of an image and finding its centroid.
"""

import cv2
import numpy as np


def classifier_parameters():
    """
    Returns two sets of thresholds indicating the three lower thresholds
    and upper thresholds for detecting pixels belonging to the track with
    respect to the background.
    """
    # This implementation is a stub. Modify with your thresholds.

    threshold_low = (0, 0, 0)
    threshold_high = (0, 0, 0)
    return threshold_low, threshold_high


def image_line_vertical(img, x_line):
    """ Adds a green 3px vertical line to the image """
    cv2.line(img, (x_line, 0), (x_line, img.shape[1]), (0, 255, 0), 3)
    return img


def image_segment(img, threshold_low, threshold_high):
    """
    Take an image and two bounds value as the input, return a second image
    indicating which pixels fall in the given bounds.
    """
    # This implementation is a stub. You should implement your own code here.

    # return img_segmented


def image_mix(img_object, img_background, threshold_low, threshold_high):
    """
    Given an image of an object (a robot) on a colored background, and a new
    background image as inputs, output a new image that combines the two by
    - Using image_segment() on img_object.
    - Creates a new image, of the same dimensions as img_object, such that
      - Every negative pixel (i.e., which is not part of the background) is
        kept equal to the corresponding one from img_object.
      - Every positive pixel (i.e., which is part of the background) is taken
        from the corresponding one in img_background.
    """
    # This implementation is a stub. You should implement your own code here.

    # return img_mix


def image_one_to_three_channels(img):
    """ Transforms an image from two channels to three channels """
    # First promote array to three dimensions,
    # then repeat along third dimension
    img_three = np.tile(img.reshape(img.shape[0], img.shape[1], 1), (1, 1, 3))
    return img_three


def image_centroid_horizontal(img):
    """
    Compute the horizontal centroid of white pixels in
    a grascale image
    """
    # Assumes that img contains only black and white pixels

    # This implementation is a stub. Substitute with your own code.
    x_centroid = 0
    return x_centroid


def pixel_count(img):
    """
    Count how many pixels in the image are non-zero (positive label)
    and zero (negative label)
    """
    nb_positive = np.count_nonzero(img)
    nb_negative = img.size - nb_positive
    return nb_positive, nb_negative


def image_centroid_test():
    # load test image
    img = cv2.imread('line-test.png')
    # make segmented image
    tlow, thigh = classifier_parameters()
    img_seg = image_segment(img, tlow, thigh)
    # compute centroid
    x_centroid = image_centroid_horizontal(img_seg)
    # make img color
    color = image_one_to_three_channels(img_seg)
    # add line on color img
    line = image_line_vertical(color, x_centroid)
    # show images
    cv2.imshow('test_original', img)
    cv2.waitKey()
    cv2.imshow('test_segmented', line)
    cv2.waitKey()


if __name__ == '__main__':
    image_centroid_test()
