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


pub_nao_action = rospy.Publisher('joint_angles', JointAnglesWithSpeed, queue_size=1)

current_target = "/face_0"
action2target = {"looks_tablet":"/tablet", "looks_child_head":"/face_0", "looks_experimentator":"/experimenter", "looks_selection_tablet":"/selection_tablet"}
mimic = False

def head_move_msg(yaw, pitch, speed):
    msg = JointAnglesWithSpeed()
    h = Header()
    h.stamp = rospy.Time.now()
    msg.header = h
    msg.joint_names = ["HeadYaw", "HeadPitch"]
    msg.joint_angles = [yaw, pitch]
    msg.speed = speed
    return msg

def onReceiveAction(msg):
    global current_target
    action = str(msg.data)
    if action in action2target:
        current_target = action2target[action]
        mimic = False
    else:
        mimic = True

if __name__=="__main__":

    rospy.init_node("nao_actions")

    """ Nao Bridge : """

    listener = tf.TransformListener()
    while True:
        test  = listener.getFrameStrings()
        rospy.Subscriber('robot_action_topic', String, onReceiveAction)

        if "robot_head" in test and "face_0" in test:
            rospy.loginfo("frames found !!!!!!")
            if not mimic:
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

                msg = head_move_msg(yaw, pitch, 0.3)
                pub_nao_action.publish(msg)

            else:
                (pose,rot) = listener.lookupTransform('/base_footprint','/face_0', rospy.Time(0))
                euler = tf.transformations.euler_from_quaternion(rot)
                roll = euler[1]
                pitch = euler[0] - np.pi/2.
                yaw = -np.sign(euler[2])*(np.abs(euler[2])-np.pi/2.)
                rospy.loginfo("pitch "+str(pitch))
                rospy.loginfo("yaw "+str(yaw))
                msg = head_move_msg(yaw, pitch, 0.2)
                pub_nao_action.publish(msg)

        rospy.sleep(0.1)
    rospy.spin()
