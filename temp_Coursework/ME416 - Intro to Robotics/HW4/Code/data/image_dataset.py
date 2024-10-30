"""
Provided image dataset
"""


import cv2


BLUE = (255, 0, 0)

train_pos = cv2.imread('/home/jilin/rastic_ws/src/me416_lab/data/line-train.png')
cross_pos = cv2.imread('/home/jilin/rastic_ws/src/me416_lab/data/line-cross-1.png')
test_pos = cv2.imread('/home/jilin/rastic_ws/src/me416_lab/data/line-test.png')
train_neg = cv2.imread('/home/jilin/rastic_ws/src/me416_lab/data/line-train.png')
cross_neg = cv2.imread('/home/jilin/rastic_ws/src/me416_lab/data/line-cross-1.png')
test_neg = cv2.imread('/home/jilin/rastic_ws/src/me416_lab/data/line-test.png')

train_pos_copy = train_pos.copy()
cross_pos_copy = cross_pos.copy()
test_pos_copy = test_pos.copy()
train_neg_copy = train_neg.copy()
cross_neg_copy = cross_neg.copy()
test_neg_copy = test_neg.copy()

cv2.rectangle(train_pos_copy, (149, 0), (168, 239), BLUE)
cv2.imshow('train_pos_copy', train_pos_copy)
cv2.rectangle(cross_pos_copy, (153, 0), (174, 239), BLUE)
cv2.imshow('cross_pos_copy', cross_pos_copy)
cv2.rectangle(test_pos_copy, (158, 0), (174, 239), BLUE)
cv2.imshow('test_pos_copy', test_pos_copy)
cv2.rectangle(train_neg_copy, (0, 0), (120, 239), BLUE)
cv2.imshow('train_neg_copy', train_neg_copy)
cv2.rectangle(cross_neg_copy, (182, 0), (319, 239), BLUE)
cv2.imshow('cross_neg_copy', cross_neg_copy)
cv2.rectangle(test_neg_copy, (0, 0), (135, 239), BLUE)
cv2.imshow('test_neg_copy', test_neg_copy)

cv2.waitKey(0)
cv2.destroyAllWindows()

train_pos_save = train_pos[0:240, 149:169]
cv2.cvtColor(src=train_pos_save, dst=train_pos_save, code=cv2.COLOR_BGR2HSV)
cv2.imwrite('train_positive.png', train_pos_save)
cross_pos_save = cross_pos[0:240, 153:175]
cv2.cvtColor(src=cross_pos_save, dst=cross_pos_save, code=cv2.COLOR_BGR2HSV)
cv2.imwrite('cross_positive.png', cross_pos_save)
test_pos_save = test_pos[0:240, 158:175]
cv2.cvtColor(src=test_pos_save, dst=test_pos_save, code=cv2.COLOR_BGR2HSV)
cv2.imwrite('test_positive.png', test_pos_save)
train_neg_save = train_neg[0:240, 0:121]
cv2.cvtColor(src=train_neg_save, dst=train_neg_save, code=cv2.COLOR_BGR2HSV)
cv2.imwrite('train_negative.png', train_neg_save)
cross_neg_save = cross_neg[0:240, 182:320]
cv2.cvtColor(src=cross_neg_save, dst=cross_neg_save, code=cv2.COLOR_BGR2HSV)
cv2.imwrite('cross_negative.png', cross_neg_save)
test_neg_save = test_neg[0:240, 0:136]
cv2.cvtColor(src=test_neg_save, dst=test_neg_save, code=cv2.COLOR_BGR2HSV)
cv2.imwrite('test_negative.png', test_neg_save)
