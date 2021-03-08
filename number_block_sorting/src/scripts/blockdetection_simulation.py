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

    objectPositions = np.array([[0.0,0,0],[0,0,0],[0,0,0],[0,0,0]])
    colors = np.array(["yellow", "green", "red", "blue"])

    def __init__(self):

        self.bridge = CvBridge() 



        self.sub_image = rospy.wait_for_message("/head_camera/rgb/image_raw", Image)
        self.pointCloudBlocks = rospy.wait_for_message("/head_camera/depth_registered/points", PointCloud2)


    #finds the X, Y, and Z coordinates from one pixel in a PointCloud2 message
    def findXYZ(self, pixelX, pixelY):

        index = (pixelY*self.pointCloudBlocks.row_step) + (pixelX*self.pointCloudBlocks.point_step)
        
        (X, Y ,Z) = struct.unpack_from('fff', self.pointCloudBlocks.data, offset=index)

        print(X, Y, Z)

        #adjusted centers of an object from the bounding rectangle:
        #Z = Z + (w/2)

        return (X, Y, Z)
        

    #finds the centers of objects in an image
    def findObjects(self, openCVImage):
        
        kernel = np.ones((5,5),np.uint8)

        deNoisedImage = cv2.fastNlMeansDenoisingColored(openCVImage, None, 10 , 10)
        ih, iw, ic = openCVImage.shape
        hsv = cv2.cvtColor(deNoisedImage, cv2.COLOR_BGR2HSV)

        hsvMedianBlur = cv2.medianBlur(hsv, 5)
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
        low_yellow = np.array([15,100,100])
        high_yellow = np.array([35,255,255])
        yellowMask = cv2.inRange(hsvMedianBlur, low_yellow, high_yellow)

        #pink
        low_blue = np.array([90, 100, 100])
        high_blue = np.array([125, 255, 255])
        blueMask = cv2.inRange(hsvMedianBlur, low_blue, high_blue)

        #creation of a mask with all objects.
        totalMask = cv2.bitwise_or(greenMask, yellowMask)
        totalMask = cv2.bitwise_or(totalMask, redMask3)
        totalMask = cv2.bitwise_or(totalMask, blueMask)

        #noise reduction on each mask
        erodedBlueMask = cv2.morphologyEx(blueMask, cv2.MORPH_OPEN, kernel)
        erodedRedMask = cv2.morphologyEx(redMask3, cv2.MORPH_OPEN, kernel)
        erodedYellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernel)
        erodedGreenMask = cv2.morphologyEx(greenMask, cv2.MORPH_OPEN, kernel)

        #calculation of the pixel center of each block in the image
        maskArray = np.array([erodedYellowMask, erodedGreenMask, erodedRedMask, erodedBlueMask])
        centers = np.array([[0, 300, 300], [1, 300, 300], [2, 300, 300], [3, 300, 300]])
        cv2.waitKey(0)
        idx = 0
        
        edges = np.array([[0,0], [0,0]])

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

                    if (idx == 0):
                        edges[0][0] = x+1
                        edges[0][1] = y-1 + (h/2)

                        edges[1][0] = x+1 + w
                        edges[1][0] = y-1 + (h/2)

                    print(self.colors[idx])
                    print(cx, cy)
                    #print(cx - icx, cy - icy)
                    centers[idx][1] = int(cx)
                    centers[idx][2] = int(cy)
                    #centers[idx][3] = w
            idx = idx + 1


        i = 0
        for x in centers:
            print(x[1], x[2])
            test = np.array(x[1],x[2])
            (X, Y, Z) = self.findXYZ(x[1], x[2])

            prePose = PoseStamped()
            prePose.pose.position.x = X
            prePose.pose.position.y = Y
            prePose.pose.position.z = Z

            outputPose = self.transform_pose(prePose, "head_camera_rgb_optical_frame", "base_link")

            self.objectPositions[i][0] = outputPose.pose.position.x
            self.objectPositions[i][1] = outputPose.pose.position.y
            self.objectPositions[i][2] = outputPose.pose.position.z
            i = i + 1
        print("BASE_LINK POSITIONS: YELLOW, GREEN, RED, PINK")
        print(self.objectPositions)
        
        return self.colors, centers

    #translates the Image message to a format usable by OpenCV
    def translateImage(self):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.sub_image, "bgr8")
        except CvBridgeError:
            rospy.logerr("CvBridge Error: {0}".format(CvBridgeError))
        
        return cv_image

    
    #transforms a pose from one pose to another pose
    def transform_pose(self, input_pose, from_frame, to_frame):

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose.pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        return output_pose_stamped

    def findDepthAdjustment(self, leftSidePixel, rightSidePixel, from_frame, to_frame):

        (leftX, leftY, leftZ) = self.findXYZ(leftSidePixel)

        (rightX, rightY, rightZ) = self.findXYZ(rightSidePixel)

        leftPose = PoseStamped()
        leftPose.pose.position.x = leftX
        leftPose.pose.position.y = leftY
        leftPose.pose.position.z = leftZ

        rightPose = PoseStamped()
        rightPose.pose.position.x = rightX
        rightPose.pose.position.y = rightY
        rightPose.pose.position.z = rightZ

        leftPoseTransformed = self.transform_pose(leftPose, from_frame, to_frame)

        rightPoseTransformed = self.transform_pose(rightPose, from_frame, to_frame)

        zAdjustment = leftPoseTransformed.pose.position.x - rightPoseTransformed.pose.position.x
        
        print("\n\n\n Z Adjustment", zAdjustment, "\n\n\n\n")

        return zAdjustment

        