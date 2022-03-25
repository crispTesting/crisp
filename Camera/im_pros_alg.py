#############
#
#   IMAGE PROCESSING FOR CIRSP
#
############
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
from utilities import *


# Absolute path to image
path = r'C:\Users\bjorn\Documents\CriSp\images\test_images\test_3.jpeg'

# Read in the image and store in a variable
img = cv2.imread(path)

# Setting up a kernel for erosion and dilation
kernel = np.ones((12,12), np.uint8)
kernel_2 = np.ones((3,3), np.uint8)

# Dimensions used for cropiing the image
crop_dimensions = (320, 240)

# lower and upper threshold of the part considered the edge of the track.
lower_threshold = (70,70,70)
higher_threshold = (110, 110, 110) 

# Blurring the image to remove som noise
blur = cv2.blur(img,(5,5))
#blur = cv2.GaussianBlur(img, (10,10), 0) 

#
#
#
# grey 0 triangle binarization
img_2 = resize(img.copy(), crop_dimensions)
grey = cv2.cvtColor(img_2, cv2.COLOR_BGR2GRAY)
gaussian = cv2.GaussianBlur(grey, (7,7), 1)
ret1, th1 = cv2.threshold(gaussian, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_TRIANGLE)
er = erode(th1, kernel_2)
dl = dilate(th1, kernel_2)
#cv2.imshow("Triangle g", gaussian)
#cv2.imshow("Triangle binarization", dl)
#
#
#

# Thresholding the image using the lower_threshold, and upper_threshold
thresh_2 = cv2.inRange(blur, lower_threshold, higher_threshold)
thresh = cv2.inRange(img, lower_threshold, higher_threshold)

# Applying erosion to the image.
img_erosion = erode(thresh, kernel, 1)

# Applying dilation to the image.
img_dilation = dilate(img_erosion, kernel, 1)

# Performing Canny Edge detection
canny = cv2.Canny(img_dilation, 200, 400)

# Resizing images
(resize_original, 
resize_blur, 
resize_thresh, 
resize_img_dilation, 
resize_canny) = resize_multiple(img, blur, thresh, 
                                img_dilation, canny, 
                                dimensions=crop_dimensions)

# Getting the height of the image
height = resize_thresh.shape[0]

# Getting horizontals
first_horizontal = int(height * 0.8) 
second_horizontal = int( height * 0.4)
third_horizontal = int (height * 0.2)

# Resizing images
#resize_original = cv2.resize(img, (320, 240))
##resize_blur = cv2.resize(blur, (320, 240)) 
#resize_thresh = cv2.resize(thresh, (320, 240))
#resize_img_dilation = cv2.resize(img_dilation, (320, 240)) 
#resize_canny = cv2.resize(canny, (320, 240)) 

# Making sure all images have three chanels in order to be displayed
thresh_3_channel = cv2.cvtColor(resize_thresh, cv2.COLOR_GRAY2BGR)
img_dialation_3_channel = cv2.cvtColor(resize_img_dilation, cv2.COLOR_GRAY2BGR)
canny_3_channel = cv2.cvtColor(resize_canny, cv2.COLOR_GRAY2BGR)

print(resize_thresh.shape)

# Finding in horizontals
first_lane = find_lane(first_horizontal, resize_thresh) 
second_lane = find_lane(second_horizontal, resize_thresh)
third_lane = find_lane(third_horizontal, resize_thresh)

# Finding the start and end of edges in each lane
first_lane_start_end = find_start_end_of_lane(first_lane)
second_lane_start_end = find_start_end_of_lane(second_lane)
third_lane_start_end = find_start_end_of_lane(third_lane)

# Counts the number of time a gap apperas in the white pixels
breaks_1 = number_of_jumps(first_lane)
breaks_2 = number_of_jumps(second_lane)
breaks_3 = number_of_jumps(third_lane)

# prints the number of times a gap apperas in the white pixels
print(f"1:\t{breaks_1}\t2:\t{breaks_2}\t{3:}\t{breaks_3}")

# Printing matrices of lanes, and printing the values of start-end of edges.
print(f"[First horizontal]:\t{first_lane_start_end}")
print(f"[Second horizontal]:\t{second_lane_start_end}")
print(f"[Third horizontal]:\t{third_lane_start_end}")

#print(img_dialation_3_channel[72][second_horizontal])

# Drawing the horizontals on a copy of the original image
img_horizontals = resize_original.copy()
cv2.line(img_horizontals, (1, first_horizontal), (303, first_horizontal), color=(255, 0, 0), thickness=2)
cv2.line(img_horizontals, (65, second_horizontal), (245, second_horizontal), color=(0, 255, 0), thickness=2)
cv2.line(img_horizontals, (53, third_horizontal), (213, third_horizontal), color=(0, 0, 255), thickness=2)


# Stacking the images horizontally
# If you want to  stack them vertically, change to "axis = 0".
horizontal_stack_1 = np.concatenate((resize_original, resize_blur, thresh_3_channel, img_dialation_3_channel, canny_3_channel, img_horizontals), axis=0)
#horizontal_stack_2 = np.concatenate((img_dialation_3_channel, canny_3_channel, img_horizontals), axis=0)

# Displaying the images
cv2.imshow("stack", horizontal_stack_1)
#cv2.imshow("stack2", horizontal_stack_2)

cv2.waitKey(0)
cv2.destroyAllWindows() 


#### RANDOM NOTES ####

# pixels between inner edges of lane.
#
# 1 horizontal:
# 2 horisontal: 182
# 3 horisontal: 74