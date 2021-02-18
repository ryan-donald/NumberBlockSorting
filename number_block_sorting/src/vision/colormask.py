#detection of colors in an image to detect multiple colored blocks.
#Ryan Donald UML PeARL February 2021

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import cv2
import numpy as np
import struct
from cv_bridge import CvBridge, CvBridgeError



bridge = CvBridge() 
#rawImage = cv2.imread('/home/ryan/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/vision/rawImage.jpg')


#ROS SUBSCRIBER FOR A SINGLE IMAGE
rospy.init_node('ImageSubscriber', anonymous=True)
rospy.loginfo("ImageSubscriber Initialized")

#sub_image = rospy.Subscriber("/head_camera/rgb/image_raw", Image, image_callback)
sub_image = rospy.wait_for_message("/head_camera/rgb/image_raw", Image)

pointCloudBlocks = rospy.wait_for_message("/head_camera/depth_registered/points", PointCloud2)

index = (264*pointCloudBlocks.row_step) + (348*pointCloudBlocks.point_step)

(X, Y ,Z) = struct.unpack_from('fff', pointCloudBlocks.data, offset=index)

print((X,Y,Z))

#points = pc2.read_points(pointCloudBlocks, field_names = ("x", "y", "z"), skip_nans = True, uvs = [249, 235])

#print(points)

print(pointCloudBlocks.height)
print(pointCloudBlocks.width)

try:
    cv_image = bridge.imgmsg_to_cv2(sub_image, "bgr8")
except CvBridgeError:
    rospy.logerr("CvBridge Error: {0}".format(CvBridgeError))


#cv2.imshow("test", rawImage)
#cv2.waitKey(9)
rawImage = cv_image
print(rawImage.shape)
cv2.waitKey(0)
#rawImage = cv2.imread('/home/ryan/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/vision/feb12.jpg')
#rawImage2 = rawImage.copy()

cv2.imshow('Original Image', rawImage)
cv2.waitKey(2)

deNoisedImage = cv2.fastNlMeansDenoisingColored(rawImage, None, 10 , 10)

cv2.imshow('DeNoised', deNoisedImage)

kernel = np.ones((5,5),np.uint8)

ih, iw, ic = rawImage.shape
icx = iw/2
icy = iw/2

hsv = cv2.cvtColor(deNoisedImage, cv2.COLOR_BGR2HSV)
cv2.imshow('HSV Image',hsv)
cv2.waitKey(0)

hsvMedianBlur = cv2.medianBlur(hsv, 5)
#color masks

#green
low_green = np.array([35,40,40])
high_green = np.array([80,255,255])
greenMask = cv2.inRange(hsvMedianBlur, low_green, high_green)

#red
low_red = np.array([0, 180,30])
high_red = np.array([10,255,255])
redMask = cv2.inRange(hsvMedianBlur,low_red, high_red)


low_red2 = np.array([150,180,30])
high_red2 = np.array([180,255,255])
redMask2 = cv2.inRange(hsvMedianBlur,low_red2, high_red2)

#yellow
low_yellow = np.array([15,80,80])
high_yellow = np.array([35,255,255])
yellowMask = cv2.inRange(hsvMedianBlur, low_yellow, high_yellow)

redMask3 = cv2.bitwise_or(redMask, redMask2)

_, contoursY, hierarchy = cv2.findContours(yellowMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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

low_pink1 = np.array([135, 100, 100])
high_pink1 = np.array([180, 170, 255])

#low_pink2 = np.array([0, 50, 60])
#high_pink2 = np.array([15, 170, 255])

#pinkMask2 = cv2.inRange(hsvMedianBlur, low_pink2, high_pink2)
#pinkMask1 = cv2.inRange(hsvMedianBlur, low_pink1, high_pink1)

#pinkMask = cv2.bitwise_or(pinkMask1,pinkMask2)
pinkMask = cv2.inRange(hsvMedianBlur, low_pink1, high_pink1)

totalMask = cv2.bitwise_or(greenMask, yellowMask)
totalMask = cv2.bitwise_or(totalMask, redMask3)
totalMask = cv2.bitwise_or(totalMask, pinkMask)

erodedPinkMask = cv2.morphologyEx(pinkMask, cv2.MORPH_OPEN, kernel)
erodedRedMask = cv2.morphologyEx(redMask3, cv2.MORPH_OPEN, kernel)
erodedYellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernel)
erodedGreenMask = cv2.morphologyEx(greenMask, cv2.MORPH_OPEN, kernel)

#yellowMaskMedian = cv2.medianBlur(yellowMask, 5)
#redMaskMedian = cv2.medianBlur(redMask3, 5)
#greenMaskMedian = cv2.medianBlur(greenMask)
colors = np.array(["yellow", "green", "red", "pink"])
maskArray = np.array([erodedYellowMask, erodedGreenMask, erodedRedMask, erodedPinkMask])
centers = np.array([["yellow", 0, 0], ["green", 0, 0], ["red", 0, 0], ["pink", 0, 0]])
cv2.waitKey(0)
idx = 0
for mask in maskArray:
    temp = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    _, contoursMask, hierarchy = cv2.findContours(temp, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contoursMask:
        area = cv2.contourArea(contour)
        if area > 300:
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
cv2.imshow("redmask1", redMask)
cv2.imshow("redmask2", redMask2)
cv2.imshow("Green Mask", erodedGreenMask)
#cv2.imshow("Red Mask", redMask)
cv2.imshow("Yellow Mask", erodedYellowMask)
#cv2.imshow("Red Mask 2", redMask2)
cv2.imshow("Red Mask Comb", erodedRedMask)
cv2.imshow("Pink Mask", erodedPinkMask)
cv2.imshow("totalmask", totalMask)

erodedTotalMask = cv2.morphologyEx(totalMask, cv2.MORPH_OPEN, kernel)

cv2.imshow("Erosion Total Mask", erodedTotalMask)
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

_, contours, hierarchy = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

_, contours2, hierarchy2 = cv2.findContours(totalMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
idx = 0
objectPositions = np.array([[0,0],[0,0],[0,0],[0,0]])
for contour in contour_list:
    x,y,w,h = cv2.boundingRect(contour)
    cv2.rectangle(mask, (x, y), (x+w, y+h), (255,0,255), 2)
    cx = x + (w/2)
    cy = y + (h/2)
    print(colors[idx])
    print(cx, cy)
    print(cx - icx, cy - icy)
    objectPositions[idx][0] = int(cx)
    objectPositions[idx][1] = int(cy)
    idx = idx + 1

print(objectPositions)

cv2.drawContours(rawImage, contour_list,  -1, (255,0,0), 2)
cv2.imshow('Objects Detected',rawImage)

#With the two arrays of object positions, it is possible to match color positions to the
#other method of detecting objects, for a stronger and more robust position detection.

cv2.waitKey(0)
