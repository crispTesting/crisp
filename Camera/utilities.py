import numpy as np
import cv2

# Find all white pixels that is the lane.
def find_lane(horizontal, image):
    
    height = image.shape[0]
    width = image.shape[1]

    lane = 255

    white_pixels = []

    for i in range( horizontal, horizontal+1 ):
        for j in range( 0, width-1 ):
            if image[horizontal][j] == lane and image[horizontal][j] == lane and image[horizontal][j] == lane:
                white_pixels.append(j)
    return white_pixels

# Finds out how many times the distance between two pixels is to great to be counted as the same line.
def number_of_jumps(white_pixels):
    if white_pixels == None:
        return 0

    count = 0
    for i in range(len(white_pixels) -1):
         if ((white_pixels[i+1] - white_pixels[i]) > 5):
             count = count + 1

    return count
        

# Need to work more on this one. Do not properly find the start and end of the lane sometimes.
def find_start_end_of_lane(white_pixels):
    left_start = 0
    left_end = 0
    right_start = 0
    right_end = 0

    print(white_pixels)

    if len(white_pixels) == 0:
        return None

    for i in range(len(white_pixels)-1):

        if left_start == 0:
            print(f"IM here\t{i}\t{i+1}\t{white_pixels[i]}\t{white_pixels[i+1]}")
            print(white_pixels[i+1] - white_pixels[i])
            left_start = white_pixels[i]
        elif white_pixels[i] >= ( white_pixels[i-1] + 10 ) and left_end == 0 and right_start == 0 and left_start != 0:
            left_end = white_pixels[i-1]
            right_start = white_pixels[i]
        elif left_start != 0 and left_end != 0 and right_start != 0:  
            if ( (len(white_pixels)-1) != i ):  

                if (white_pixels[i+1] - white_pixels[i]) > 10:
                    c = white_pixels[i+1] - white_pixels[i]
                    right_end = white_pixels[i]
                    print(f"HELLO....\t{i}\t{c}\t{right_end}")  

    return [left_start, left_end, right_start, right_end]

# Warping the image... I have to think how the image should be warped.
def warp(img):
    width, height, c = img.shape
    #pts1 = np.float32([[111,219],[287,188],[154,482],[352,440]])
    pts1 = np.float32([[0,45],[240,45],[0,240],[600,240]])
    pts2 = np.float32([[0,0],[width,0],[0,height],[width,height]])
    matrix = cv2.getPerspectiveTransform(pts1,pts2)
    imgOutput = cv2.warpPerspective(img,matrix,(width,height))
    return imgOutput


def erode(img, kernel, iter=1):
    return cv2.erode(img, kernel, iterations=iter)

def dilate(img, kernel, iter=1):
    return cv2.dilate(img, kernel, iter)

# Resizing a variable number of images
def resize_multiple(*images, dimensions):
    resized = []

    for img in images:
        resized.append(resize(img, dimensions))
    
    return resized

# Resizing and cropping an image
def resize(img, dimension_tuple):
    resized = cv2.resize(img, dimension_tuple)
    height, width = resized.shape[:2]
    h = int(height/2)
    h2 = int(height/6)
    cropped = resized[h2:h, :width]
    return cropped




'''
# Warping the image
warp_img = warp(resize_original)
warp_blur = cv2.blur(warp_img,(5,5))
warp_thresh = cv2.inRange(warp_blur, lower_threshold, higher_threshold)
warp_e = cv2.erode(warp_thresh, kernel, iterations=1)
warp_d = cv2.dilate(warp_e, kernel, iterations=1)
warp_c = cv2.Canny(warp_d, 200, 400)

warp_thresh_3 = cv2.cvtColor(warp_thresh, cv2.COLOR_GRAY2BGR)
warp_d_3 = cv2.cvtColor(warp_d, cv2.COLOR_GRAY2BGR)
warp_c_3 = cv2.cvtColor(warp_c, cv2.COLOR_GRAY2BGR)

horizontal_stack_2 = np.concatenate((warp_img, warp_blur, warp_thresh_3, warp_d_3, warp_c_3), axis=1)
'''

