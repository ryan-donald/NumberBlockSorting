#! /usr/bind/env python

#Ryan Donald

#This script contains the grasping class, which includes multiple functions used in the 
#overall task of object detection with multiple pick and place manipulations to sort blocks.

import rospy
import numpy as np
import actionlib

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_python import PlanningSceneInterface, PickPlaceInterface, MoveGroupInterface
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal



class Grasping(object):

    def __init__(self):
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.scene = PlanningSceneInterface("base_link")
        self.move_group = MoveGroupInterface("arm", "base_link")

        find_objects = "basic_grasping_perception/find_objects"
        
        self.find_client = actionlib.SimpleActionClient(find_objects, FindGraspableObjectsAction)
        self.find_client.wait_for_server()


    def pickup(self, block, grasps):

        success, pick_result = self.pickplace.pick_with_retry(block.name, grasps, support_name=block.support_surface, scene = self.scene)

        self.pick_result = pick_result
        return success

    #swaps two blocks positions.
    def swapBlockPos(self, block1Pos, block2Pos):
        #intermediate point for movement

        self.armIntermediatePose()

        #intermediate positon on the table, used for short term storage of the first block manipulated in the sorting.
        posIntermediate = np.array([0.725,0])
        
        self.armIntermediatePose()

        # Get block to pick
        while not rospy.is_shutdown():
            rospy.loginfo("Picking object...")
            self.updateScene()
            cube, grasps = self.getGraspableCube(block1Pos)
            if cube == None:
                rospy.logwarn("Perception failed.")
                continue

            # Pick the block
            if self.pickup(cube, grasps):
                break
            rospy.logwarn("Grasping failed.")

        #self.tuck()
        #Place the block
        while not rospy.is_shutdown():
            rospy.loginfo("Placing object...")
            pose = PoseStamped()
            pose.pose = cube.primitive_poses[0]
            pose.pose.position.z += 0.05
            pose.header.frame_id = cube.header.frame_id
            if self.place(cube, pose, posIntermediate):
                break
            rospy.logwarn("Placing failed.")
        
        #place block 2 in block 1's
        self.armIntermediatePose()
         # Get block to pick
        while not rospy.is_shutdown():
            rospy.loginfo("Picking object...")
            self.updateScene()
            cube, grasps = self.getGraspableCube(block2Pos)
            if cube == None:
                rospy.logwarn("Perception failed.")
                continue

            # Pick the block
            if self.pickup(cube, grasps):
                break
            rospy.logwarn("Grasping failed.")

        while not rospy.is_shutdown():
            rospy.loginfo("Placing object...")
            pose = PoseStamped()
            pose.pose = cube.primitive_poses[0]
            pose.pose.position.z += 0.05
            pose.header.frame_id = cube.header.frame_id
            if self.place(cube, pose, block1Pos):
                break
            rospy.logwarn("Placing failed.")

        #place block1 in block 2's spot
        self.armIntermediatePose()
        
        while not rospy.is_shutdown():
            rospy.loginfo("Picking object...")
            self.updateScene()
            cube, grasps = self.getGraspableCube(posIntermediate)
            if cube == None:
                rospy.logwarn("Perception failed.")
                continue

            # Pick the block
            if self.pickup(cube, grasps):
                break
            rospy.logwarn("Grasping failed.")

        #self.tuck()
        self.armIntermediatePose()

        while not rospy.is_shutdown():
            rospy.loginfo("Placing object...")
            pose = PoseStamped()
            pose.pose = cube.primitive_poses[0]
            pose.pose.position.z += 0.05
            pose.header.frame_id = cube.header.frame_id
            if self.place(cube, pose, block2Pos):
                break
            rospy.logwarn("Placing failed.")

        return

    def place(self, block, pose_stamped, placePos):

        #creates a list of place positons for the block, with a specified x, y, and z.

        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = "base_link"

        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat

        #this x and y value are input as placePos through the function call.
        l.place_pose.pose.position.x = placePos[0]
        l.place_pose.pose.position.y = placePos[1]
        

        places.append(copy.deepcopy(l))

        #success = self.pickplace.place_with_retry(block.name, places, scene = self.scene)

        #return success


        ## create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success



    #This function planes and executes movement of the gripper to a specified (x, y, z) coordinate in the frame "base_link"
    def gripperToPosition(self, x, y, z):
        
        #new pose_stamped of the end effector that moves the arm out of the way of the vision for planning.
        intermediatePose = PoseStamped()

        intermediatePose.header.frame_id = 'base_link'

        #position
        intermediatePose.pose.position.x = x
        intermediatePose.pose.position.y = y
        intermediatePose.pose.position.z = z

        #quaternion for the end position

        intermediatePose.pose.orientation.w = 1


        while not rospy.is_shutdown():
            result = self.move_group.moveToPose(intermediatePose,"wrist_roll_link")
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    
    def armIntermediatePose(self):

        self.armForward(0.1,-0.7,0.9)

    def armForward(self, x, y, z):
        
        #new pose_stamped of the end effector that moves the arm out of the way of the vision for planning.
        intermediatePose = PoseStamped()

        intermediatePose.header.frame_id = 'base_link'

        #position
        intermediatePose.pose.position.x = x
        intermediatePose.pose.position.y = y
        intermediatePose.pose.position.z = z

        #quaternion for the end position

        intermediatePose.pose.orientation.w = 1


        while not rospy.is_shutdown():
            result = self.move_group.moveToPose(intermediatePose,"wrist_roll_link")
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return


    #FROM DEMO.PY TO BE REPLACED
    def updateScene(self):
        # find objectsw
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        idx = -1
        for obj in find_result.objects:
            idx += 1
            obj.object.name = "object%d"%idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0],
                                         use_service = False)

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            1.5,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0],
                                         use_service = True)

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getGraspableCube(self, pos):
        graspable = None

        for obj in self.objects:
            # need grasps

            if len(obj.grasps) < 1:
                continue


            # check size
            if obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07:


                continue
            # has to be on table
            if (obj.object.primitive_poses[0].position.z < 0.3) or \
                (obj.object.primitive_poses[0].position.y > pos[1]+0.05) or \
                (obj.object.primitive_poses[0].position.y < pos[1]-0.05) or \
                (obj.object.primitive_poses[0].position.x > pos[0]+0.05) or \
                (obj.object.primitive_poses[0].position.x < pos[0]-0.05):
                continue

            return obj.object, obj.grasps
        # nothing detected
        return None, None

    #END FROM DEMO.PY
