#detection of colors in an image to detect multiple colored blocks.

import cv2
import numpy as np



rawImage = cv2.imread('/home/ryan/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/vision/2021-02-07-165326.jpg')
rawImage2 = rawImage.copy()
cv2.imshow('Original Image', rawImage)
cv2.waitKey(0)

ih, iw, ic = rawImage.shape
icx = iw/2
icy = iw/2

hsv = cv2.cvtColor(rawImage, cv2.COLOR_BGR2HSV)
cv2.imshow('HSV Image',hsv)
cv2.waitKey(0)

hsvMedianBlur = cv2.medianBlur(hsv, 5)
#color masks

#green
low_green = np.array([35,40,40])
high_green = np.array([80,255,255])
greenMask = cv2.inRange(hsvMedianBlur, low_green, high_green)

#red
low_red = np.array([0,120,111])
high_red = np.array([10,255,255])
redMask = cv2.inRange(hsvMedianBlur,low_red, high_red)


low_red2 = np.array([170,120,111])
high_red2 = np.array([180,255,255])
redMask2 = cv2.inRange(hsvMedianBlur,low_red2, high_red2)

#yellow
low_yellow = np.array([10,100,100])
high_yellow = np.array([30,255,255])
yellowMask = cv2.inRange(hsvMedianBlur, low_yellow, high_yellow)

redMask3 = cv2.bitwise_or(redMask, redMask2)

contoursY, hierarchy = cv2.findContours(yellowMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#idx = 1
#for contour in contoursY:
#    x,y,w,h = cv2.boundingRect(contour)
#    cv2.rectangle(yellowMask, (x, y), (x+w, y+h), (255,0,255), 2)
#    print("Yellow Object ", idx, ": x : ", x, " y: ", y, " w: ", w, " h: ", h)
#    cx = x + (w/2)
#    cy = y + (h/2) 
#    print("center: (", cx,",", cy,")" )
#    idx = idx + 1


#pink

low_pink = np.array([160, 20, 150])
high_pink = np.array([175, 130, 255])
pinkMask = cv2.inRange(hsvMedianBlur, low_pink, high_pink)


totalMask = cv2.bitwise_or(greenMask, yellowMask)
totalMask = cv2.bitwise_or(totalMask, redMask3)
totalMask = cv2.bitwise_or(totalMask, pinkMask)

#yellowMaskMedian = cv2.medianBlur(yellowMask, 5)
#redMaskMedian = cv2.medianBlur(redMask3, 5)
#greenMaskMedian = cv2.medianBlur(greenMask)
colors = np.array(["yellow", "green", "red", "pink"])
maskArray = np.array([yellowMask, greenMask, redMask3, pinkMask])
centers = np.array([["yellow", 0, 0], ["green", 0, 0], ["red", 0, 0], ["pink", 0, 0]])
idx = 0
for mask in maskArray:
    contoursMask, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contoursMask:
        area = cv2.contourArea(contour)
        if area > 150:
            x,y,w,h = cv2.boundingRect(contour)
            cv2.rectangle(mask, (x, y), (x+w, y+h), (255,0,255), 2)
            cx = x + (w/2)
            cy = y + (h/2)
            print(colors[idx])
            print(cx, cy)
            print(cx - icx, cy - icy)
            centers[idx][1] = int(cx)
            centers[idx][2] = int(cy)
    idx = idx + 1

print(centers)

cv2.imshow("Green Mask", greenMask)
#cv2.imshow("Red Mask", redMask)
cv2.imshow("Yellow Mask", yellowMask)
#cv2.imshow("Red Mask 2", redMask2)
cv2.imshow("Red Mask Comb", redMask3)
cv2.imshow("Pink Mask", pinkMask)
cv2.imshow("totalmask", totalMask)
cv2.waitKey(0)

#output = cv2.bitwise_and(rawImage, rawImage, mask = greenMask)
#cv2.imshow("Green", np.hstack([rawImage, output]))

#cv2.waitKey(0)


hue ,saturation ,value = cv2.split(hsvMedianBlur)
cv2.imshow('Saturation Image',saturation)
cv2.waitKey(0)

retval, thresholded = cv2.threshold(saturation, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
cv2.imshow('Thresholded Image',thresholded)
cv2.waitKey(0)

contours, hierarchy = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

contours2, hierarchy2 = cv2.findContours(totalMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

contour_list = []
for contour in contours:
    area = cv2.contourArea(contour)
    if area > 100 :
        contour_list.append(contour)

contour_list2 = []

for contour in contours2:
    area = cv2.contourArea(contour)
    if area > 100 :
        contour_list2.append(contour)
#cv2.drawContours(rawImage2, contour_list2, -1, (0,255,0), 2)
#cv2.imshow('Contours 2', rawImage2)

#for contour in contours:
#    x,y,w,h = cv2.boundingRect(contour)
#    cv2.rectangle(rawImage, (x,y), (x+w, y+h), (255,0,255),2)
#    print("x: ", x, " y: ", y, " w: ", w, " h: ",h )

cv2.drawContours(rawImage, contour_list,  -1, (255,0,0), 2)
cv2.imshow('Objects Detected',rawImage)
cv2.waitKey(0)
