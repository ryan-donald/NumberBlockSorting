import numpy as np
import cv2
import os
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

img1 = cv2.imread('/home/ryan/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/vision/Bike.jpg', 0)
img2 = cv2.imread('/home/ryan/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/vision/Vehicles2.jpg', 0)
#robotImage = rospy.wait_for_message("topic", Image)
#bridge = CvBridge()

#try:
#    img2 = bridge.imgmsg_to_cv2(robotImage, "mono8")
#except CvBridgeError:
#    rospy.logerr("CvBridge Error: {0}".format(CvBridgeError))


cv2.imshow("testestest", img2)
cv2.waitKey(0)
cv2.imshow("TESTEST", img1)
cv2.waitKey(0)

# Initiate SIFT detector
sift = cv2.xfeatures2d.SIFT_create()

# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

# FLANN parameters
#FLANN_INDEX_KDTREE = 0
#index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
#search_params = dict(checks=50)   # or pass empty dictionary

#flann = cv2.FlannBasedMatcher(index_params,search_params)

#matches = flann.knnMatch(des1,des2,k=2)

bf = cv2.BFMatcher()
matches = bf.knnMatch(des1, des2, k=2)

# Need to draw only good matches, so create a mask
matchesMask = [[0,0] for i in xrange(len(matches))]

# ratio test as per Lowe's paper
goodMatches = []
for i,(m,n) in enumerate(matches):
    if m.distance < 0.7*n.distance:
        matchesMask[i]=[1,0]
        goodMatches.append(matches[i])


draw_params = dict(matchColor = (0,255,0),
                   singlePointColor = (255,0,0),
                   matchesMask = matchesMask,
                   flags = 0)

img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)

xSum = 0
ySum = 0

for match in goodMatches:
    #finds the keypoints in the captured image.
    a = kp2[match[0].trainIdx].pt
    b = kp1[match[0].queryIdx].pt

    xSum = xSum + a[0]
    ySum = ySum + a[1]

    #print(match)
    print("train")
    print(a)
    print("query")
    print(b)



xSum = int(xSum) / len(goodMatches)
ySum = int(ySum) / len(goodMatches)
        
print(xSum)
print(ySum)

plt.imshow(img3,),plt.show()