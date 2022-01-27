#!/usr/bin/env python

####                                                                ####
#                                                                      #
#   This is script is for developing the lane detection algorithm      #
#                                                                      #
####                                                                ####
import cv2
import numpy as np

path_img_one = r'C:\Users\bjorn\Documents\CriSp\track_mounted_higher.png'
path_img_two = r'C:\Users\bjorn\Documents\CriSp\track_mouunted_down.png'
path_img_three = r'C:\Users\bjorn\Documents\CriSp\test.png'
path_img_four = r'C:\Users\bjorn\Documents\CriSp\road4.jpeg'

def region_of_interest(image):
    height = image.shape[0]
    width = image.shape[1]
    triangle = [np.array([
        (10, height), 
        (width-10, height), 
        (int(width/2), int(height/2))
    ])]
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, triangle, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

img_one = cv2.imread(path_img_two)
window_one = "Higher"


# Thresholding
grey_one = cv2.cvtColor(img_one, cv2.COLOR_BGR2GRAY)
#cv2.imshow("Gray one", grey_one)

# Binary
thresh = 127
im_bw = cv2.threshold(grey_one, thresh, 255, cv2.THRESH_BINARY_INV)[1]
#cv2.imshow("binary", im_bw)

# Bluring
grey_one = cv2.blur(grey_one, (7, 7))
#cv2.imshow("Blur one", grey_one)

# Canny edges
edges_one = cv2.Canny(grey_one, 16.0, 30.0, apertureSize = 3)
#cv2.imshow("Canny binary", edges_one)

cropped = region_of_interest(edges_one)
cv2.imshow("Cropped", cropped)

'''
# Hough transform
lines = cv2.HoughLines(cropped,1,np.pi/180,200)
for line in lines:
    for rho,theta in line:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        cv2.line(img_one ,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imshow("houghgie", img_one)
'''

minLineLength = 100 
maxLineGap = 9
lines = cv2.HoughLinesP(cropped,1,np.pi/180,100,minLineLength,maxLineGap)
for line in lines:
    for x1,y1,x2,y2 in line:
        cv2.line(img_one,(x1,y1),(x2,y2),(0,255,0),2)

 
#cv2.imwrite(r'C:\Users\bjorn\Documents\CriSp\houghlines3.jpg',img_one) 
cv2.imshow("hough transform", img_one)

contours, hierarchy = cv2.findContours(cropped , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

for contour in contours:
    c = abs(contour[0][0][0]-contour[0][0][1])
    if  c < 60:
        print(contour)
        cv2.drawContours(img_one, contour, -1, (0, 255, 0), 3)
cv2.imshow("contours", img_one)

cv2.waitKey(0)
cv2.destroyAllWindows() 