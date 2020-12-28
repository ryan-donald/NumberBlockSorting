#! /usr/bin/env python

#Ryan Donald 11/20/2020. 
# This script accounts for the translation of an object for this project,
# ALL VISION IS TO BE REPLACED BY ANOTHER SCRIPT LATER, ONLY FOR TESTING PURPOSES. CODE LIKELY HAS BUGS 

import demo
import copy
import rospy
import actionlib
import numpy as np
#import pyperplantranslate.py as pplt

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_python import PlanningSceneInterface, PickPlaceInterface
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#class used to store the current positions of the objects. Used in sorting to check if there is an object in the wanted position.
class objectPositions():

    #booleans for if an object is stored in a specific spot or not. 0 = empty, last index is the intermediate spot
    objects = np.array([0, 0, 0, 0, 0])

    def objectInPos(self, testPos):

        if objects[testPos] != "":
            return True
        else:
            return False

    def storeObjects(self, objects):
        self.objects = np.copy(objects)
        self.objects.append([0])


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
        posIntermediate = np.array([0.8,0.35])


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


        # Place the block
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
        self.tuck()
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
        self.tuck()
        
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

        self.tuck()

        while not rospy.is_shutdown():
            rospy.loginfo("Placing object...")
            pose = PoseStamped()
            pose.pose = cube.primitive_poses[0]
            pose.pose.position.z += 0.05
            pose.header.frame_id = cube.header.frame_id
            if self.place(cube, pose, block2Pos):
                break
            rospy.logwarn("Placing failed.")

    def place(self, block, pose_stamped, placePos):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = "base_link"

        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat

        l.place_pose.pose.position.x = placePos[0]
        l.place_pose.pose.position.y = placePos[1]
        

        places.append(copy.deepcopy(l))
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

    #FROM DEMO.PY TO BE REPLACED
    def updateScene(self):
        # find objects
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
                                         obj.object.primitive_poses[0])

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
                                         obj.primitive_poses[0])

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass


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
    
    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return
    #END FROM DEMO.PY

if __name__ == "__main__":

    rospy.init_node("translation")

    #position array that contains the min and max values for the y coordinate of each position (1,2,3,4) 
    posY = np.array([[0.2, 0.3], [0.05, 0.15], [-0.15,-0.05], [-0.3,-0.2]])

    #intermediate point for movement
    posIntermediate = np.array([0.7,0.0])

    #array that contains the possible positions of the blocks, to be replaced with vision defined spots
    posPlaces = np.array([[0.8, 0.25], [0.8,0.075], [0.8,-0.075], [0.8,-0.25]])

    currentPos = np.array([[]])

    grasping_class = Grasping()
    rospy.loginfo("Grasping Class Initialized")



    #symbolicPlanner = pplt.PyperPlanTranslation()

    #for loop for interpretation of pyperplan solution
    #for x in symbolicPlanner.commands:
    #    if "place" in symbolicPlanner.commands:
    #        #place block in hand
    #    elif "pick-up" in symbolicPlanner.commands:
    #        #pickup block specified
    #    elif "sort" in symbolicPlanner.commands:
    #        #sort the two blocks, destination zone in the left of the block

    
    

    #FROM DEMO.PY
    torso_action = demo.FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    rospy.loginfo("FollowTrajectoryClient Class Initialized")
    head_action = demo.PointHeadClient()
    rospy.loginfo("PointHeadClient Class Initialized")

    rospy.loginfo("Beginning manipulation...")


    head_action.look_at(0.8, 0, 0.43, "map")

    grasping_class.swapBlockPos(posPlaces[1],posPlaces[3])

    
