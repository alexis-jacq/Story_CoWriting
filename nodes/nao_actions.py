#!/usr/bin/env python
#coding: utf-8

import sys
import time
import numpy as np
import random

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Empty, Header
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
import tf

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

"""
NAO_IP = "10.0.0.17"
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
"""

pub_nao_action = rospy.Publisher('joint_angles', JointAnglesWithSpeed, queue_size=1)

def head_move_msg(yaw, pitch, speed):
    msg = JointAnglesWithSpeed()
    h = Header()
    h.stamp = rospy.Time.now()
    msg.header = h
    msg.joint_names = ["HeadYaw", "HeadPitch"]
    msg.joint_angles = [yaw, pitch]
    msg.speed = speed
    return msg

#def onReceiveAction(msg):

if __name__=="__main__":

    rospy.init_node("nao_actions")

    """ Nao Bridge : """

    """
    testTime = 5 # seconds
    t = 0
    dt = 0.2
    while (t<testTime):
        yaw = random.uniform(-1.0, 1.0)
        pitch = random.uniform(-0.5, 0.5)
        msg = head_move_msg(yaw, pitch, 0.6)
        pub_nao_action.publish(msg)
        t = t + dt
        time.sleep(dt)
    """



    listener = tf.TransformListener()
    while True:
        test  = listener.getFrameStrings()
        #rospy.loginfo(test)

        if "robot_head" in test and "selection_tablet" in test:
            rospy.loginfo("frames found !!!!!!")
            (pose,rot) = listener.lookupTransform('/robot_head','/selection_tablet' , rospy.Time(0))
            x = pose[0]
            y = pose[1]
            z = pose[2]
            rospy.loginfo("x "+str(x))
            rospy.loginfo("y "+str(y))
            rospy.loginfo("z "+str(z))
            yaw = np.sign(z)*np.arccos(y)
            pitch = np.sign(y)*np.arccos(x)
            rospy.loginfo("pitch "+str(pitch))
            rospy.loginfo("yaw "+str(yaw))
            if np.random.rand()>0.5:
                msg = head_move_msg(0., 0., 0.6)
                pub_nao_action.publish(msg)
            else:
                msg = head_move_msg(yaw/2., 0., 0.6)
                pub_nao_action.publish(msg)

            """yaw = random.uniform(-1.0, 1.0)
            pitch = random.uniform(-0.5, 0.5)
            msg = head_move_msg(yaw, pitch, 0.6)
            pub_nao_action.publish(msg)"""

        rospy.sleep(0.5)
    rospy.spin()

    """ Naoqi :
    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)

    testTime = 10 # seconds
    t = 0
    dt = 0.2

    # start face tracking:
    targetName = "Face"
    faceWidth = 0.1
    tracker.registerTarget(targetName, faceWidth)
    tracker.track(targetName)

    while (t<testTime):
        #motionProxy.setAngles("HeadYaw", random.uniform(-1.0, 1.0), 0.6)
        #motionProxy.setAngles("HeadPitch", random.uniform(-0.5, 0.5), 0.6)

        t = t + dt
        time.sleep(dt)

    # stop face tracking:
    tracker.stopTracker()
    tracker.unregisterAllTargets()
    motionProxy.rest()
    """
