"""
Consider the following pairs of images:
1) ’train_positive.png’ , ’train_negative.png’ ;
2) ’cross_positive.png’ , ’cross_negative.png’ .

Write a Python script or use IPython to run the function pixel_count ( ) on each image
and report:
• Number of points for each image.
• Total number of positives and negatives for each image.
• Number of false positives and false negatives for each image (for this, you need to
consider that files ending in _positive have been cropped to contain only positive
examples, and files ending in _negative only negative examples).
• Precision and recall for each pair.

Include the images and the values in your report using a table so that they can be easily
interpreted.
"""


import cv2
import numpy as np
import image_processing as ip


train_pos = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/data/train_positive.png")
train_neg = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/data/train_negative.png")
cross_pos = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/data/cross_positive.png")
cross_neg = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/data/cross_negative.png")
test_pos = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/data/test_positive.png")
test_neg = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/data/test_negative.png")

targets = [train_pos, train_neg, cross_pos, cross_neg, test_pos, test_neg]
targets_names = ["train_positive.png", "train_negative.png",
                 "cross_positive.png", "cross_negative.png",
                 "test_positive.png", "test_negative.png"]
low, high = ip.classifier_parameters()
TRAIN_TRUE_POS = 4800.
CROSS_TRUE_POS = 5280.
TEST_TRUE_POS = 4080.
recall = 0
precision = 0
false_neg = 0
false_pos = 0

print(f"{'-':-^50}")
for ii, target in enumerate(targets):
    print(f"Current image is: {targets_names[ii]}")
    target_segmented = ip.image_segment(target, low, high)
    pos, neg = ip.pixel_count(target_segmented)
    print(f"Total number of points: {pos+neg}")
    print(f"Total number of positive points: {pos}")
    print(f"Total number of negative points: {neg}")
    if targets_names[ii] == "train_positive.png" \
        or targets_names[ii] == "cross_positive.png" \
        or targets_names[ii] == "test_positive.png":
        print(f"Number of false negatives: {neg}")
        false_neg = neg
    else:
        print(f"Number of false positives: {pos}")
        false_pos = pos
    print(f"{'-':-^50}")
    if ii%2 == 1: # one pair has been calculated
        if targets_names[ii] == "train_negative.png":
            recall = TRAIN_TRUE_POS/(TRAIN_TRUE_POS+false_neg)
            precision = TRAIN_TRUE_POS/(TRAIN_TRUE_POS+false_pos)
            print(f"'train_' pair recall = {recall}, precision = {precision}")
            print(f"{'-':-^50}")
        elif targets_names[ii] == "cross_negative.png":
            recall = CROSS_TRUE_POS/(CROSS_TRUE_POS+false_neg)
            precision = CROSS_TRUE_POS/(CROSS_TRUE_POS+false_pos)
            print(f"'cross_' pair recall = {recall}, precision = {precision}")
            print(f"{'-':-^50}")
        else:
            recall = TEST_TRUE_POS/(TEST_TRUE_POS+false_neg)
            precision = TEST_TRUE_POS/(TEST_TRUE_POS+false_pos)
            print(f"'test_' pair recall = {recall}, precision = {precision}")
            print(f"{'-':-^50}") 
