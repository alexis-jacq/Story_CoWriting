#!/usr/bin/env python
#coding: utf-8

import sys
import time
import numpy as np
import random

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Empty, Header
#from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
import tf

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule


NAO_IP = "192.168.1.66"
port = 9559
myBroker = ALBroker("myBroker", #
    "0.0.0.0",   # listen to anyone
    0,           # find a free port and use it
    NAO_IP,      # parent broker IP
    port)        # parent broker port
hasFallen = False
motionProxy = ALProxy("ALMotion", NAO_IP, port)
memoryProxy = ALProxy("ALMemory", NAO_IP, port)
postureProxy = ALProxy("ALRobotPosture", NAO_IP, port)
faceProxy = ALProxy("ALFaceDetection", NAO_IP, port)
tracker = ALProxy("ALTracker", NAO_IP, port)

def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


"""pub_nao_action = rospy.Publisher('joint_angles', JointAnglesWithSpeed, queue_size=1)"""

current_target = "/face_0"
action2target = {"looks_tablet":"/tablet", "looks_child_head":"/face_0", "looks_experimentator":"/experimenter", "looks_selection_tablet":"/selection_tablet"}
mimic = True#False

"""def head_move_msg(yaw, pitch, speed):
    msg = JointAnglesWithSpeed()
    h = Header()
    h.stamp = rospy.Time.now()
    msg.header = h
    msg.joint_names = ["HeadYaw", "HeadPitch"]
    msg.joint_angles = [yaw, pitch]
    msg.speed = speed
    return msg"""

def onReceiveAction(msg):
    global current_target
    action = str(msg.data)
    '''if action in action2target:
        current_target = action2target[action]
        mimic = False
    else:
        mimic = True'''

if __name__=="__main__":

    rospy.init_node("nao_actions")

    """ Nao Bridge or naoqi : """
    StiffnessOn(motionProxy)

    listener = tf.TransformListener()
    listener.waitForTransform('/base_footprint','/face_0', rospy.Time(0), rospy.Duration(4.0))

    while True:
        test  = listener.getFrameStrings()
        rospy.Subscriber('robot_action_topic', String, onReceiveAction)

        if "base_footprint" in test and "robot_head" in test and "face_0" in test:
            rospy.loginfo("frames found !!!!!!")
            #if not mimic:
            if False:
                (pose,rot) = listener.lookupTransform('/robot_head', current_target, rospy.Time(0))
                x = pose[0]
                y = pose[1]
                z = pose[2]
                rospy.loginfo("x "+str(x))
                rospy.loginfo("y "+str(y))
                rospy.loginfo("z "+str(z))
                yaw = np.arctan(y/x)
                pitch = np.arctan(-z/x)
                rospy.loginfo("pitch "+str(pitch))
                rospy.loginfo("yaw "+str(yaw))

                # that can be done directly via naoqi
                """msg = head_move_msg(yaw, pitch, 0.3)
                pub_nao_action.publish(msg)"""

                motionProxy.setAngles("HeadYaw", yaw, 0.3)
                motionProxy.setAngles("HeadPitch", pitch, 0.3)

            else:
                listener.waitForTransform('/base_footprint','/face_0', rospy.Time(0), rospy.Duration(4.0))
                (pose,rot) = listener.lookupTransform('/base_footprint','/face_0', rospy.Time(0))
                euler = tf.transformations.euler_from_quaternion(rot)
                roll = euler[1]
                pitch = euler[0] - np.pi/2.
                yaw = -np.sign(euler[2])*(np.abs(euler[2])-np.pi/2.)
                rospy.loginfo("pitch "+str(pitch))
                rospy.loginfo("yaw "+str(yaw))
                
                # that can be done directly via naoqi
                """msg = head_move_msg(yaw, pitch, 0.2)
                pub_nao_action.publish(msg)"""
                
                motionProxy.setAngles("HeadYaw", yaw, 0.3)
                motionProxy.setAngles("HeadPitch", pitch, 0.3)


        rospy.sleep(0.1)

    #motionProxy.rest()
    rospy.spin()

