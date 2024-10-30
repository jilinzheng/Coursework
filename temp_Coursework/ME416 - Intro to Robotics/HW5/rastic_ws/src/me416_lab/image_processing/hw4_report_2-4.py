"""
Write a Python script or use IPython to call the function
image_segment() on the images line-training.png, line-cross.png.
Visualize the results using the function cv2.imshow (),
and the function cv2.imwrite() to save them to files
(you can choose any file name, please refer to a past in-class activity on how to use
these functions). Include the images in the report.
"""


import cv2
import image_processing as ip


line_train = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/data/line-train.png")
line_cross = cv2.imread("/home/jilin/rastic_ws/src/me416_lab/data/line-cross-1.png")

# convert to HSV since that is what my parameters are based off of
cv2.cvtColor(src=line_train, dst=line_train, code=cv2.COLOR_BGR2HSV)
cv2.cvtColor(src=line_cross, dst=line_cross, code=cv2.COLOR_BGR2HSV)

low, high = ip.classifier_parameters()
line_train_segmented = ip.image_segment(line_train, low, high)
line_cross_segmented = ip.image_segment(line_cross, low, high)
print(line_train_segmented)
print(type(line_train_segmented))

cv2.imshow("line_train_segmented", line_train_segmented)
cv2.imshow("line_cross_segmented", line_cross_segmented)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imwrite("line_train_segmented.png", line_train_segmented)
cv2.imwrite("line_cross_segmented.png", line_cross_segmented)
