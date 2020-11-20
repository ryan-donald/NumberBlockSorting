#! /usr/bin/env python

#Ryan Donald 11/20/2020
# File used to create a text file to store joint information

import rospy
from sensor_msgs.msg import JointState

run = 1

def callback(data):
    global run
    joints = open("TestRunData.txt", "a")
    joints.write("Time of run: " + str(data.header.time) + "\n\n")
    joints.write("Joint Names:\n" + str(data.name) + "\n")
    joints.write("Joint positions:\n" + str(data.position) + "\n")
    joints.write("Joint velocities:\n" + str(data.velocity) + "\n\n")
    joints.close()
    run += 1

def subscribe_joint_states():

    rospy.init_node('jointstorage', anonymous=True)

    rospy.Subscriber('joint_states', JointState, callback)
    
    rospy.spin()


if __name__ == '__main__':
    joints = open("TestRunData.txt", "w")
    joints.close()
    subscribe_joint_states()