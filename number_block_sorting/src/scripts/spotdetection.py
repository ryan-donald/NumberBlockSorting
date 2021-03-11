import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class spotDetection():

    bridge = CvBridge() 
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters_create()

    #array containing the corners of each space in relation to the image. Useful for calculation of the place point for MoveIT!
    #spaces = np.array([0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0])


    #finds the poses for each Ar Marker representing a valid space for the block to be in.
    def findSpacesAR(self):

        self.poseList = []

        self.poseList.append(rospy.wait_for_message("/ar_pose_marker", AlvarMarkers))

        while true:
            
            arPoseMessage = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)

            for x in poseList:
                if x.id == arPoseMessage.id:
                    continue:
                else:
                    poseList.append(arPoseMessage)
    
    #ids is a list of the spot ar marker id's with an increasing value, ex. spot 1, spot 2, spot 3.
    #The last ID is the id of the intermediate position
    def findSpotOrder(self, ids):

        orderedList = []


        for x in ids:

            for y in self.poseList:

                if (y.id == x):
                    orderedList.append(y)

        return orderedList
        
    def findSpaces(self, image):
        
        cv2.imshow("IMAGE", image)
        cv2.waitKey(0)

        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)

        if len(corners) > 0:

            ids = ids.flatten()

            for (markerCorner, markerID) in zip(corners, ids):

                corners = markerCorner.reshape((4,2))

                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(image, topLeft, topRight, (0,255,0), 2)
                cv2.line(image, topRight, bottomRight, (0,255,0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0,255,0), 2)
                cv2.line(image, bottomLeft, topLeft, (0,255,0), 2)

                cX = int((topLeft[0] + bottomRight[0] / 2.0))
                cY = int((topLeft[1] + bottomRight[1] / 2.0))

                cv2.circle(image, (cX, cY), 4, (0,0,25), -1)

                cv2.putText(image, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: ()".format(markerID))

                cv2.imshow("Image", image)
                cv2.waitKey(0)


        


    def translateImage(self, rosImage):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
        except CvBridgeError:
            rospy.logerr("CvBridge Error: {0}".format(CvBridgeError))
        
        return cv_image
        
        
if __name__ == "__main__":

    #image = cv2.imread('/home/ryan/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/vision/IMG_1593.jpg')

    rospy.init_node('ImageSubscriber', anonymous=True)
    rospy.loginfo("ImageSubscriber Initialized")

    #sub_image = rospy.Subscriber("/head_camera/rgb/image_raw", Image, image_callback)
    sub_image = rospy.wait_for_message("/head_camera/rgb/image_raw", Image)

    test = spotDetection()

    raw_image = test.translateImage(sub_image)

    test.findSpaces(raw_image)