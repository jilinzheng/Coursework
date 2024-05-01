"""
Homework 4 Report 2.8

Write a Python script to call the function image_mix ( ) to combine
all possible combinations of, separately, the training images, and testing images.
You should end up with eight new combined images.
Include all the original and obtained images in your report
(organizing them in a clear way).
"""


import cv2
import image_processing as ip


def get_var_name(var):
    for name, value in globals().items():
        if value is var:
            return name

robot_training_1 = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/image_processing/robot_training_1.png")
robot_training_2 = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/image_processing/robot_training_2.png")
background_training_1 = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/image_processing/background_training_1.jpg")
background_training_2 = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/image_processing/background_training_2.jpg")

robot_testing_1 = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/image_processing/robot_testing_1.png")
robot_testing_2 = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/image_processing/robot_testing_2.png")
background_testing_1 = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/image_processing/background_testing_1.jpg")
background_testing_2 = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/image_processing/background_testing_2.jpg")

img_mix_pairs = [[robot_training_1, background_training_1],
                 [robot_training_1, background_training_2],
                 [robot_training_2, background_training_1],
                 [robot_training_2, background_training_2],
                 [robot_testing_1, background_testing_1],
                 [robot_testing_1, background_testing_2],
                 [robot_testing_2, background_testing_1],
                 [robot_testing_2, background_testing_2]]
bg_threshold_low = (30, 0, 30)
bg_threshold_high = (255, 255, 255)


for pair in img_mix_pairs:
    img_mixed = ip.image_mix(pair[0], pair[1], bg_threshold_low, bg_threshold_high)
    cv2.imshow(f"{get_var_name(pair[0])}+{get_var_name(pair[1])}", img_mixed)
    cv2.imwrite(f"{get_var_name(pair[0])}+{get_var_name(pair[1])}.png", img_mixed)

cv2.waitKey(0)
cv2.destroyAllWindows()
