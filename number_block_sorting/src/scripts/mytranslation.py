#! /usr/bin/env python

#Ryan Donald 11/20/2020. 
# This script accounts for the translation of an object for this project,
# ALL VISION IS TO BE REPLACED BY ANOTHER SCRIPT LATER, ONLY FOR TESTING PURPOSES. CODE LIKELY HAS BUGS 

import demo
import copy
import rospy
import actionlib
import numpy as np
import pyperplantranslate as pplt
import re
import blockdetection as vision
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

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Class for internal storage of the objects positions at a current time. This will be automatically filled by the vision system. This allows for the robot to know what position a 
# block is in to grab that specific block for the manipulation actions from the pyperplan solution.
class objectPositions():

    # array for storage of the blocks. The values refer to the block, and the index refers to the position in line, left to right, on the table.
    objects = np.array([1, 4, 3, 2])
    colors = np.array("","","","")
    # returns what object is in the given position.
    def objectInPos(self, testPos, testValue):

        if self.objects[testPos] == testValue:
            return True
        else:
            return False

    # function to load the objects array from vision.
    def storeObjects(self, objects):
        self.objects = np.copy(objects)
        self.objects.append([0])

    # returns the index of a specified value.
    def posOfObject(self, num):
        idx = 0
        for x in self.objects:
            if x == num:
                return idx
            
            idx = idx + 1
            

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

    def armIntermediatePose(self):

        self.armForward(0.1,-0.7,0.9)
        

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
    posIntermediate = np.array([0.6,0.0])

    #array that contains the possible positions of the blocks, to be replaced with vision defined spots
    posPlaces = np.array([[0.6, 0.25], [0.6,0.075], [0.6,-0.075], [0.6,-0.25]])

    currentPos = np.array([[]])

    grasping_class = Grasping()

    objectPos = objectPositions()


    vision_class = vision.ObjectDetection()

    image = vision_class.translateImage()

    colors, points = vision_class.findObjects(image)

    #i = 0
    #for x in points:
    #    (X, Y, Z) = vision_class.findXYZ(x)

    #    j = 0
    #    for x in posPlaces:
    #        if (np.aboslute(x[0] - X) < 0.05) & (np.absolute(x[1] - Y) < 0.05):
    #            objectPos.colors[j] = colors[i]
    #        j = j + 1
    #    i = i + 1





    rospy.loginfo("Grasping Class Initialized")

    symbolicPlanner = pplt.PyperPlanTranslation()

    symbolicPlanner.InterpretSolution()

    #FROM DEMO.PY
    torso_action = demo.FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    rospy.loginfo("FollowTrajectoryClient Class Initialized")
    head_action = demo.PointHeadClient()
    rospy.loginfo("PointHeadClient Class Initialized")

    rospy.loginfo("Beginning manipulation...")

    

    #used to position the robots vision camera in a fixed position that can accurately view the entire workspace.
    head_action.look_at(0.65, 0, 0.43, "map")

    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])

    grasping_class.armIntermediatePose()
    

    test1 = np.array[vision_class.objectPositions[0][0], vision_class.objectPositions[0][1]]
        # Get block to pick
    while not rospy.is_shutdown():
        rospy.loginfo("Picking object...")
        self.updateScene()
        cube, grasps = self.getGraspableCube(test1)
        if cube == None:
            rospy.logwarn("Perception failed.")
            continue

        # Pick the block
        if self.pickup(cube, grasps):
            break
        rospy.logwarn("Grasping failed.")


    return


    for x in symbolicPlanner.commands:
        rospy.loginfo("forloopworks")
        temp = x.replace("(","")
        temp = temp.replace(")","")
        temp = temp.split()
        if temp[0] == "sort":
            
            
            #calls swapBlockPos on the two positions of the specified blocks in from the pyperplan solution. Eg: block 2 and block 4
            grasping_class.swapBlockPos(
                posPlaces[objectPos.posOfObject(int(temp[1]))],
                posPlaces[objectPos.posOfObject(int(temp[2]))])

            #Swaps the positions in the code of the two blocks that were just swapped.
            y = objectPos.objects[objectPos.posOfObject(int(temp[1]))]
            objectPos.objects[objectPos.posOfObject(int(temp[1]))] = objectPos.objects[objectPos.posOfObject(int(temp[2]))]
            objectPos.objects[objectPos.posOfObject(int(temp[2]))] = y

            #returns the arm to the intermediate pose.
            #grasping_class.armIntermediatePose()
        #elif "place" in x:
            #grasping_class.place()
        #elif "pick-up" in x:
            # Get block to pick
            #while not rospy.is_shutdown():
            #    rospy.loginfo("Picking object...")
            #    self.updateScene()
            #    cube, grasps = self.getGraspableCube(posPlaces(objectPos.posOfObject(num(0))))
            #    if cube == None:
            #    rospy.logwarn("Perception failed.")
            #    continue

            # Pick the block
            #if self.pickup(cube, grasps):
            #    break
            #rospy.logwarn("Grasping failed.")

        #end if statements


    #end for loop    
    #grasping_class.swapBlockPos(posPlaces[1],posPlaces[3])

