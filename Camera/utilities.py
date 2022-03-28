import numpy as np
import cv2

def scan_for_lane_mid(horizontal, img):
    '''
        Finds the inner boundaries of the lane.

        Parameters:
        --------------------
        horizontal (int) : The row number that should be scanned in the image.
        img (Image) : The image to scan on.

        Returns:
        --------------------
        left_lane (int) : The pixel column for where the left lane starts.
        right_lane (int) : The pixel column for where the right lane starts.

    '''
    width = img.shape[1]
    LANE = 255

    left_lane = 0
    right_lane = 0
    mid = int(width/2)

    # Finding start of left lane. Scaning from middle to left.
    for i in range(mid, 0, -1):
        if img[horizontal][i] == LANE and left_lane == 0:
            left_lane = i

    # Finding start of right lane. Scanning from middle to right.
    for j in range(mid, width, 1):
        if img[horizontal][j] == LANE and right_lane == 0:
            right_lane = j

    return left_lane, right_lane

def process_image(img, crop_dimensions, kernel):
    '''
        Does all the preprocessing of the image before calculations can be done.

        First the image is resized. Then its made into greyscale. Then a gaussian filter is applied.
        After that we apply a triangular binarization. Finally I apply a dilation to fill in the gaps.

        Parameters:
        --------------------
        img (img) : The image to process
        crop_dimensions (tuple of ints) : The dimensions to use for cropping
        kernel (tuple of ints) : The kernel to be used for dilation.

        Returns:
        dilated (img) : The finnished processed image
    '''
    img_resized = resize(img, crop_dimensions)
    grey = cv2.cvtColor(img_resized, cv2.COLOR_BGR2GRAY)
    gaussian = cv2.GaussianBlur(grey, (7,7), 1)
    ret1, th1 = cv2.threshold(gaussian, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_TRIANGLE)
    dilated = dilate(th1, kernel)
    return dilated

def erode(img, kernel, iter=1):
    '''
        Erodes an image with the given kernel and iterations.

        Parameters:
        --------------------
        img (img) : The image to process
        kernel (tuple of ints) : The pixel kernel to use for erosion.
        iter (int) : The number of times the erosion filtering should be executed.

        Returns:
        --------------------
        cv2.erode(img, kernel, iterations=iter) (img) : The eroded image.
    '''
    return cv2.erode(img, kernel, iterations=iter)

def dilate(img, kernel, iter=1):
    '''
        Dilates an image with the given kernel and iterations.

        Parameters:
        --------------------
        img (img) : The image to process
        kernel (tuple of ints) : The pixel kernel to be used for dilation
        iter (int) : The number of times the dilation filtering should be executed

        Returns:
        --------------------
        cv2.dilate(img, kernel, iter) (img) : The dilated image.
    '''
    return cv2.dilate(img, kernel, iter)

# Resizing a variable number of images
def resize_multiple(*images, dimensions):
    '''
        Resizes an n number of images with the given dimensions.

        Parameters:
        --------------------
        *images (Varargs of images) : The images to resize
        dimensions (tuple of ints) : The dimensions to resize to.

        Returns resized (list) : A list of all the resized images.
    '''
    resized = []
    for img in images:
        resized.append(resize(img, dimensions))
    return resized

# Resizing and cropping an image
def resize(img, dimension_tuple):
    '''
        Resizes an image and for faster image processing and finally also crops its field of vision.

        Parameters:
        --------------------
        img (img) : The image to be resized
        dimension_tuple (tuple of ints) : The dimensions to be resized to.

        Returns:
        --------------------
        cropped (img) : The resized and cropped image
    '''
    resized = cv2.resize(img, dimension_tuple)
    height, width = resized.shape[:2]
    h = int(height/2)
    h2 = int(height/6)
    cropped = resized[h2:h, :width]
    return cropped

def draw_horizontals(img, horizontals):
    '''
        Draws horizontal lines on a image.

        Parameters:
        --------------------
        img (img) : The image to draw on
        horizontals (list) : a 2d list where each list contains of,
                            0 : the height of the horizontal line
                            1: the left lane
                            2: the right lane
        
        Returns:
        --------------------
        img (img) : The image drawn on
    '''
    # horizontals[0][0] The horizontal
    # horizontals[0][1] The left lane
    # horizontals[0][2] The right lane
    for i in range(3):
        if (horizontals[i][2]==0):
            horizontals[i][2] = horizontals[i][1]

    cv2.line(img, (horizontals[0][1], horizontals[0][0]), (horizontals[0][2], horizontals[0][0]), color=(255, 0, 0), thickness=4)
    cv2.line(img, (horizontals[1][1], horizontals[1][0]), (horizontals[1][2], horizontals[1][0]), color=(0, 255, 0), thickness=4)
    cv2.line(img, (horizontals[2][1], horizontals[2][0]), (horizontals[2][2], horizontals[2][0]), color=(0, 0, 255), thickness=4)
    return img

def draw_heading(img, horizontals, middlepoints):
    '''
        Draws the heading line on an image.

        Parameters:
        --------------------
        img (img) : The image to draw on.
        horizontals (list) : A 2d list where each list contains:
                                0:  The horizontal line
                                1:  The left lane
                                2:  The right lane

        middlepoints (list) : A list containing the middlepoints for the horizontal lines. 

        Returns:
        --------------------
        img (img) : The image drawn on.
    '''
    cv2.line(img,(middlepoints[0], horizontals[0][0]), (middlepoints[1], horizontals[1][0]), color=(255, 60, 140), thickness=2 )
    cv2.line(img,(horizontals[0][1], horizontals[0][0]), (horizontals[1][1], horizontals[1][0]), color=(255, 180, 60), thickness=2 )
    return img











