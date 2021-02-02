import numpy as np
import cv2

rawImage = cv2.imread('/home/ryan/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/vision/rawImage.jpg')
cv2.imshow('Original Image', rawImage)
cv2.waitKey(0)

hsv = cv2.cvtColor(rawImage, cv2.COLOR_BGR2HSV)
cv2.imshow('HSV Image',hsv)
cv2.waitKey(0)

hue ,saturation ,value = cv2.split(hsv)
cv2.imshow('Saturation Image',saturation)
cv2.waitKey(0)

retval, thresholded = cv2.threshold(saturation, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
cv2.imshow('Thresholded Image',thresholded)
cv2.waitKey(0)

medianFiltered = cv2.medianBlur(thresholded,5)
cv2.imshow('Median Filtered Image',medianFiltered)
cv2.waitKey(0)

contours, hierarchy = cv2.findContours(medianFiltered, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

contour_list = []
for contour in contours:
    area = cv2.contourArea(contour)
    if area > 100 :
        contour_list.append(contour)

cv2.drawContours(rawImage, contour_list,  -1, (255,0,0), 2)
cv2.imshow('Objects Detected',rawImage)
cv2.waitKey(0)