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
first_horizontal = int(height * 0.8) 
second_horizontal = int( height * 0.4)
third_horizontal = int (height * 0.2)

# Getting left and right lanes
h1_l, h1_r = scan_for_lane_mid(first_horizontal, img_processed)
h2_l, h2_r = scan_for_lane_mid(second_horizontal, img_processed)
h3_l, h3_r = scan_for_lane_mid(third_horizontal, img_processed)

# Calculating middle points for all horizontals
m1 = int( (h1_r - h1_l) / 2) + h1_l
m2 = int( (h2_r - h2_l) / 2) + h2_l
m3 = int( (h3_r - h3_l) / 2) + h3_l

print(f"First:\t{h1_l} - {h1_r}")
print(f"Second:\t{h2_l} - {h2_r}")
print(f"Third:\t{h3_l} - {h3_r}")

# List of horizonbtals and left and right lane pixels
horizontals = [[first_horizontal, h1_l, h1_r], [second_horizontal, h2_l, h2_r], [third_horizontal, h3_l, h3_r]]
# Same as above -- this should probably be refactored instead of having lot of magic code
horizontals2 = [[first_horizontal, h1_l, h1_r], [second_horizontal, h2_l, h2_r]]
# List of middlepoints
middlepoints = [m1, m2]

# This is a possible way of calculating distance and so on.
#
# Should make a picture of the track and measure how far do I see. And use that in the calculations of speed and so on.
#hypotenuse = (h2_r - h1_r) / (second_horizontal - first_horizontal)

# Drawing the horizontals
img_draw = draw_horizontals(img_org_resized, horizontals)
# Drawing a heading vector on the image.
draw_heading(img_draw, horizontals2, middlepoints)
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
