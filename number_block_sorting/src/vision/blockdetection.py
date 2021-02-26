#Detection of objects in an image to detect multiple colored blocks.
#Ryan Donald UML PeARL February 2021

import rospy
import numpy as np
import cv2
import struct
import tf2_ros
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped


#Class for detection of objects, with multiple functions for various values
class ObjectDetection():

    objectPositions = np.array([[0,0,0],[0,0,0],[0,0,0],[0,0,0]])


    def __init__(self):

        self.bridge = CvBridge() 

        self.rospy.init_node('ImageSubscriber', anonymous=True)
        rospy.loginfo("ImageSubscriber Initialized")

        self.sub_image = rospy.wait_for_message("/head_camera/rgb/image_raw", Image)
        self.pointCloudBlocks = rospy.wait_for_message("/head_camera/depth_registered/points", PointCloud2)


    #finds the X, Y, and Z coordinates from one pixel in a PointCloud2 message
    def findXYZ(self, pixel, w):

        index = (pixel[1]*self.pointCloudBlocks.row_step) + (pixel[0]*self.pointCloudBlocks.point_step)
        
        (X, Y ,Z) = struct.unpack_from('fff', self.pointCloudBlocks.data, offset=index)

        #adjusted centers of an object from the bounding rectangle:
        Z = Z + (w/2)

        return (X, Y, Z)
        

    #finds the centers of objects in an image
    def findObjects(self, openCVImage):
        
        kernel = np.ones((5,5),np.uint8)

        deNoisedImage = cv2.fastNlMeansDenoisingColored(rawImage, None, 10 , 10)
        ih, iw, ic = rawImage.shape
        hsv = cv2.cvtColor(deNoisedImage, cv2.COLOR_BGR2HSV)

        #color bounds

        #green
        low_green = np.array([35,40,40])
        high_green = np.array([80,255,255])
        greenMask = cv2.inRange(hsvMedianBlur, low_green, high_green)

        #red bounds 1
        low_red = np.array([0, 180,30])
        high_red = np.array([10,255,255])
        redMask = cv2.inRange(hsvMedianBlur,low_red, high_red)

        #red bounds 2
        low_red2 = np.array([150,180,30])
        high_red2 = np.array([180,255,255])
        redMask2 = cv2.inRange(hsvMedianBlur,low_red2, high_red2)

        #red combination mask

        redMask3 = cv2.bitwise_or(redMask, redMask2)

        #yellow
        low_yellow = np.array([15,80,80])
        high_yellow = np.array([35,255,255])
        yellowMask = cv2.inRange(hsvMedianBlur, low_yellow, high_yellow)

        #pink
        low_pink = np.array([135, 100, 100])
        high_pink = np.array([180, 170, 255])
        pinkMask = cv2.inRange(hsvMedianBlur, low_pink, high_pink)

        #creation of a mask with all objects.
        totalMask = cv2.bitwise_or(greenMask, yellowMask)
        totalMask = cv2.bitwise_or(totalMask, redMask3)
        totalMask = cv2.bitwise_or(totalMask, pinkMask)

        #noise reduction on each mask
        erodedPinkMask = cv2.morphologyEx(pinkMask, cv2.MORPH_OPEN, kernel)
        erodedRedMask = cv2.morphologyEx(redMask3, cv2.MORPH_OPEN, kernel)
        erodedYellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernel)
        erodedGreenMask = cv2.morphologyEx(greenMask, cv2.MORPH_OPEN, kernel)

        #calculation of the pixel center of each block in the image
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

        i = 0
        for x in points:
            (X, Y, Z) = vision_class.findXYZ(x)
            self.objectPositions[i][0] = X
            self.objectPositions[i][0] = Y
            self.objectPositions[i][0] = Z
            i = i + 1

        
        return colors, centers

    #translates the Image message to a format usable by OpenCV
    def translateImage(self):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.sub_image, "bgr8")
        except CvBridgeError:
            rospy.logerr("CvBridge Error: {0}".format(CvBridgeError))
        
        return cv_image


    #transforms a pose from one pose to another pose
    def transform_pose(input_pose, from_frame, to_frame):

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        return output_pose_stamped