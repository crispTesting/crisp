
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
path = r'C:\Users\bjorn\Documents\CriSp\images\t_29_march\2.jpeg'
# Loading the image from path
img = cv2.imread(path)
# Crop dimensions for faster image processing
crop_dimensions = (320, 240)
# Kernel used for dilation and erosion
kernel = np.ones((3,3), np.uint8)

# Gets the processed image
img_processed = process_image(img, crop_dimensions, kernel)
# Resized image of original for displaying
img_org_resized = resize(img, crop_dimensions)
# Copy of original, used for drawing on.
img_org_resized_2 = resize(img, crop_dimensions)


# Calculating horizontals
height = img_processed.shape[0]
first_horizontal = int(height*0.8)    # 0.8 # height-1
second_horizontal = int( height * 0.4)  # 0.4 # 0.9
third_horizontal = int (height * 0.2)   # 0.2 # 0.8

#
#fourth_horizontal = int(height * 0.7) 
#fifth_horizontal = int( height * 0.6)
#sixth_horizontal = int (height * 0.55)

#for i in range(img_processed.shape[0]):
#    for j in range(141, -1, -1):
#        img_processed[i][j] = 0

#for i in range(138, 320, 1):
#    for j in range(38):
#        img_processed[j][i] = 0

# Getting left and right lanes
h1_l, h1_r = scan_for_lane_mid(first_horizontal, img_processed)
h2_l, h2_r = scan_for_lane_mid(second_horizontal, img_processed)
h3_l, h3_r = scan_for_lane_mid(third_horizontal, img_processed)

#h4_l, h4_r = scan_for_lane_mid(fourth_horizontal, img_processed)
#h5_l, h5_r = scan_for_lane_mid(fifth_horizontal, img_processed)
#h6_l, h6_r = scan_for_lane_mid(sixth_horizontal, img_processed)

# Calculating middle points for all horizontals
m1 = int( (h1_r - h1_l) / 2) + h1_l
m2 = int( (h2_r - h2_l) / 2) + h2_l
m3 = int( (h3_r - h3_l) / 2) + h3_l


print(f"First:\t{h1_l} - {h1_r}")
print(f"Second:\t{h2_l} - {h2_r}")
print(f"Third:\t{h3_l} - {h3_r}")

# List of horizonbtals and left and right lane pixels
horizontals = [[first_horizontal, h1_l, h1_r], [second_horizontal, h2_l, h2_r], [third_horizontal, h3_l, h3_r]]
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

lateral_error = m1 - I_mid 
print(f"Lateral error:\t{lateral_error}")

I_mid_vector = [I_mid, img_processed.shape[0]]
print(f"I_mid vector:\t{I_mid_vector}")

slope = (second_horizontal-first_horizontal)/(m2-m1) 

print(f"SLOPE:\t{slope}")

heading_angle = math.atan2(first_horizontal*I_mid-m1*img_processed.shape[0], m1*I_mid+first_horizontal*img_processed.shape[0])
print(f"HEADING ANLGE:\t{math.degrees(heading_angle)}")

# Calculating the converstional constant between pixels and cm
h1_c = get_pw_to_cm_conversion_constant(h1_r, h1_l)
h2_c = get_pw_to_cm_conversion_constant(h2_r, h2_l)
h3_c = get_pw_to_cm_conversion_constant(h3_r, h3_l)
print(f"\nPW to CM:\nFirst:\tSecond:\tThird:\n{h1_c}\t{h2_c}\t{h3_c}")


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