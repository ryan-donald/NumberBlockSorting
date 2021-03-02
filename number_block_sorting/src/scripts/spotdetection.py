import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class spotDetection():

    bridge = CvBridge() 
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters_create()


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


        #find find spaces relative to base_link

        #return array with center of each space and the corners

        #


    def translateImage(self, rosImage):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
        except CvBridgeError:
            rospy.logerr("CvBridge Error: {0}".format(CvBridgeError))
        
        return cv_image
        
        
if __name__ == "__main__":

    image = cv2.imread('/home/ryan/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/vision/IMG_1592.jpg')

    test = spotDetection()

    test.findSpaces(image)