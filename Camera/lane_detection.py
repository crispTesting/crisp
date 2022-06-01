#################################################################################################################################################
#                                                                                                                                               #
#   Next steps:                                                                                                                                 #
#                                                                                                                                               #                                                                   #                                                                                                                                               #
#   Properly implement the calculation of heading angle.                                                                                        #
#                                                                                                                                               #
#   Implement storage of global variables to store last know location of the lane.                                                              #
#   -   Explore how to deal with global variabes using ROS.                                                                                     #
#                                                                                                                                               #
#   Measure the width of the lane for different scenarious and store them in a table.                                                           #
#   -   Can be stored in E.G utilis, or maybe better in a speartae file as an ENUM.                                                             #
#                                                                                                                                               #
#   Find the lateral error.                                                                                                                     #
#                                                                                                                                               #                                                                                                                                    #                                                                                                                   #
#                                                                                                                                               #
#   LAST THING TO DO:                                                                                                                           #
#   Write a comprehensive documentation of the CriSp project                                                                                    #
#   -   What has been done, thoughts of future works, improvements on existing solutions.                                                       #
#   -   Thougts around paths not taken, motivation of why this way was chosen. - Software, hardware, track design.                              #
#   -   List of materials - On the car - for the track - left in spare parts box.                                                               #
#                                                                                                                                               #
#   REMEMBER:                                                                                                                                   #
#                                                                                                                                               #
#   Must upload latest scripts to the f1tenth car, and make sure everything is working correctly.                                               #
#                                                                                                                                               #
#                                                                                                                                               #
#                                                                                                                                               #
#                                                                                                                                               #
#################################################################################################################################################


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
import math

# Absolute path to image
path = r'/home/robotlab/crisp_git/crisp/test_images/high_mount/21.jpeg'


# Loading the image from path
img = cv2.imread(path)

resize_dimensions = (427, 240)
# Kernel used for dilation and erosion
kernel = np.ones((3,3), np.uint8)

# Gets the processed image
img_processed = process_image(img, resize_dimensions, kernel)
# Resize
img_org_resized = resize(img, resize_dimensions)
# Copy of original, used for drawing on.
img_org_resized_2 = resize(img, resize_dimensions)

# Calculating horizontals
height = img_processed.shape[0]
bottom_horizontal = int(height*0.6)    # 0.8 # height-1
mid_horizontal = int( height * 0.4)  # 0.4 # 0.9
mid_horizontal_2 = int(height * 0.3)
top_horizontal = int (height * 0.2)   # 0.2 # 0.8

# Getting left and right lanes
h1_l, h1_r, h1_l_e, h1_r_e = scan_for_lane_mid(bottom_horizontal, img_processed)
h2_l, h2_r, h2_l_e, h2_r_e = scan_for_lane_mid(mid_horizontal, img_processed)
h2_l_2, h2_r_2, h2_l_e_2, h2_r_e_2 = scan_for_lane_mid(mid_horizontal_2, img_processed)
h3_l, h3_r, h3_l_e, h3_r_e = scan_for_lane_mid(top_horizontal, img_processed)


# Calculating middle points for all horizontals
m1 = int( (h1_r - h1_l) / 2) + h1_l
m2 = int( (h2_r - h2_l) / 2) + h2_l
m3 = int( (h3_r - h3_l) / 2) + h3_l

print("\tLEFT:\tLEFT END:\tRIGHT:\tRIGHT END:\tY:\tMid")
print(f"First:\t{h1_l}\t{h1_l_e}\t\t{h1_r}\t{h1_r_e}\t\t{bottom_horizontal}\t{m1}")
print(f"Second:\t{h2_l}\t{h2_l_e}\t\t{h2_r}\t{h2_r_e}\t\t{mid_horizontal}\t{m2}")
print(f"Third:\t{h3_l}\t{h3_l_e}\t\t{h3_r}\t{h3_r_e}\t\t{top_horizontal}\t{m3}")

# List of horizonbtals and left and right lane pixels
horizontals = [[bottom_horizontal, h1_l, h1_r], [mid_horizontal, h2_l, h2_r], [mid_horizontal_2, h2_l_2, h2_r_2], [top_horizontal, h3_l, h3_r]]
#horizontals = [[first_horizontal, h1_l, h1_r], [second_horizontal, h2_l, h2_r], [third_horizontal, h3_l, h3_r], [fourth_horizontal, h4_l, h4_r], [fifth_horizontal, h5_l, h5_r], [sixth_horizontal, h6_l, h6_r]]
# Same as above -- this should probably be refactored instead of having lot of magic code
#horizontals2 = [[first_horizontal, h1_l, h1_r], [second_horizontal, h2_l, h2_r]]
# List of middlepoints
middlepoints = [m1, m2]

# This is a possible way of calculating distance and so on.
#
# Should make a picture of the track and measure how far do I see. And use that in the calculations of speed and so on.
#hypotenuse = (h2_r - h1_r) / (second_horizontal - first_horizontal)

I_mid = img_processed.shape[1] / 2
#print(f"IMID:\t{I_mid}")
I_mid_vector = [I_mid, img_processed.shape[0]]
#print(f"I_mid vector:\t{I_mid_vector}")
#slope = (second_horizontal-first_horizontal)/(m2-m1) 
#print(f"SLOPE:\t{slope}")
#heading_angle = math.atan2(bottom_horizontal*I_mid-m1*img_processed.shape[0], m1*I_mid+bottom_horizontal*img_processed.shape[0])
#print(f"HEADING ANLGE ERROR:\t{math.degrees(heading_angle)}")

#h_error = get_heading_error(h1_l-h1_l_e)
#print(f"HEADING ERROR:\t{h_error}")

# Calculating the converstional constant between pixels and cm
h1_c = get_pw_to_cm_conversion_constant(h1_r, h1_l)
h2_c = get_pw_to_cm_conversion_constant(h2_r, h2_l)
h3_c = get_pw_to_cm_conversion_constant(h3_r, h3_l)
print(f"\nPW to CM:\nFirst:\tSecond:\tThird:\n{h1_c}\t{h2_c}\t{h3_c}")

get_lateral_error(I_mid, m1, h1_c)


# Drawing the horizontals
img_draw = draw_horizontals(img_org_resized, horizontals)
# Drawing a heading vector on the image.
draw_heading(img_draw, horizontals, middlepoints)
# Making the binary image have 3_channels - needed for displaying
img_processed_3_channel = cv2.cvtColor(img_processed, cv2.COLOR_GRAY2BGR)
# Stacking the images in a stack before displaying
img_stack = np.concatenate( (img_processed_3_channel, img_org_resized), axis=0)

# Using matplotlib to display the images in a nice way.
fig, axs = plt.subplots(3)
fig.suptitle('Image processing')
axs[0].imshow(img_org_resized_2)
axs[1].imshow(img_processed, cmap='gray')
axs[2].imshow(img_org_resized)
axs[0].set_title("Original image")
axs[1].set_title("Triangular binarization")
axs[2].set_title("Original image, with horizontals and heading")
fig.tight_layout()
plt.show()

cv2.waitKey(0)
cv2.destroyAllWindows()