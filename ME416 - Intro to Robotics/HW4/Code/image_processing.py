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
    # BGR COLORSPACE
    #threshold_low = (90., 165., 108.)
    #threshold_high = (175., 200., 180.)
    # HSV COLORSPACE
    threshold_low = (47, 50, 143)
    threshold_high = (71, 129, 225)
    return threshold_low, threshold_high


def image_segment(img, threshold_low, threshold_high):
    """
    Take an image and two bounds value as the input,
    return a second image indicating which pixels fall in the given bounds.
    """
    img_segmented = cv2.inRange(img, threshold_low, threshold_high)
    return img_segmented


def image_patch(img, x, y, w, h):
    """ Returns a region of interest of img specified by box """
    # check box against the boundaries of the image
    box = [y, x, y + h, x + w]
    if box[0] < 0:
        box[0] = 0
    if box[1] < 0:
        box[1] = 0
    if box[2] > img.shape[0]:
        box[2] = img.shape[0]
    if box[3] > img.shape[1]:
        box[3] = img.shape[1]

    return img[box[0]:box[2], box[1]:box[3], :]


def image_line_vertical(img, x):
    """ Adds a green 3px vertical line to the image """
    cv2.line(img, (x, 0), (x, img.shape[1]), (0, 255, 0), 3)
    return img


#def image_rectangle(img, x, y, w, h):
#    """ Adds a green rectangle to the image """
#    # This implementation is a stub. You should implement your own code here.
#
#    return img


def image_one_to_three_channels(img):
    """ Transforms an image from two channels to three channels """
    # First promote array to three dimensions,
    # then repeat along third dimension
    img_three = np.tile(img.reshape(img.shape[0], img.shape[1], 1), (1, 1, 3))
    return img_three


def image_centroid_horizontal(img):
    """
    Compute the horizontal centroid of white pixels in a grascale image.
    Assumes that img contains only black and white pixels.
    """
    # append white pixels' x-positions to list
    white_pixel_positions = []
    for row in img:
        for jj, pixel in enumerate(row):
            if pixel == 255:
                white_pixel_positions.append(jj)

    # if there are white pixels, return the median of its x-positions
    if len(white_pixel_positions) != 0:
        white_pixel_positions = np.array(white_pixel_positions)
        return int(np.median(white_pixel_positions))

    # otherwise just return 0
    return 0


def pixel_count(img):
    """
    Count how many pixels in the image are non-zero (positive label)
    and zero (negative label)
    """
    nb_positive = 0
    nb_negative = 0

    for row in img:
        for pixel in row:
            if pixel == 255:
                nb_positive+=1
            else:
                nb_negative+=1

    return nb_positive, nb_negative


def precision_recall(true_positive, false_positive, false_negative):
    """
    Return precision and recall given counts
    """
    precision = float(true_positive)/(true_positive+false_negative)
    recall = float(true_positive)/(true_positive+false_positive)
    return precision, recall


#def segmentation_statistics(filename_positive, filename_negative):
#    """
#    Print segmentation statistics for two files with known segmentation
#    """
#    precision = 0
#    recall = 0
#    return precision, recall


def image_centroid_test():
    """
    Unit test for image_centroid_horizontal()
    """
    # load test image
    img = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/data/line-test.png")
    # convert to HSV (what classifier parameters are based on)
    cv2.cvtColor(src=img, dst=img, code=cv2.COLOR_BGR2HSV)
    # make segmented image
    lb, ub = classifier_parameters()
    img_seg = cv2.inRange(img, lb, ub)
    # compute centroid
    x = image_centroid_horizontal(img_seg)
    #print(x)
    # make img color
    color = image_one_to_three_channels(img_seg)
    # add line on color img
    line = image_line_vertical(color, x)
    # show images
    cv2.imshow("test_original", img)
    #cv2.imshow("img_seg", img_seg)
    #cv2.imshow("color", color)
    cv2.imshow("test_segmented", line)
    cv2.waitKey()


def image_mix(img_object, img_background, threshold_low, threshold_high):
    """
    Given an image of an object (a robot) on a colored background,
    and a new background image as inputs,
    output a new image that combines the two
    """
    img_seg = image_segment(img_object, threshold_low, threshold_high)
    img_mix = np.empty((img_seg.shape[0], img_seg.shape[1], 3), np.uint8)

    for ii, row in enumerate(img_seg):
        for jj, pixel in enumerate(row):
            for kk in range(3):
                if pixel == 0: # neg pixels are kept equal to img_object
                    img_mix[ii, jj, kk] = img_object[ii, jj, kk]
                else: # pos pixels are taken from img_background
                    img_mix[ii, jj, kk] = img_background[ii, jj, kk]

    return img_mix


if __name__ == "__main__":
    image_centroid_test()
