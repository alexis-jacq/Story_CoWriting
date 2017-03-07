#!/usr/bin/env python
#coding: utf-8

import sys
import time
import numpy as np
import random
import rospy
import tf

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Empty, Header
from naoStoryTelling import story_gestures as sg

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule


############################################# global values
NAO_IP = "192.168.1.64" #HACK : this should not be hardcoded !
port = 9559
speed = 0.1
current_target = "/face_0"
action2target = {"tablet":"/tablet", "child_head":"/face_0", "experimentator":"/experimenter", "selection_tablet":"/selection_tablet"}
ok = False
stay = False
stop = False
point_screen = False
have_to_talk = False
have_to_talk_long = False

############################################# init proxies
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
tts = ALProxy("ALTextToSpeech", NAO_IP, port)
tts.setLanguage("English")

############################################## publisher for interface
pub_story_telling = rospy.Publisher('story_telling', String, queue_size = 1)

def story_telling(phrase):
	msg = String()
	msg.data = phrase
	pub_story_telling.publish(msg)

############################################# what if new action message
message_to_tell = ""
long_message_to_tell = ""

def onReceiveTarget(msg):
    global current_target
    action = str(msg.data)
    # gaze behavior
    if action in action2target:
        current_target = action2target[action]

def onReceiveSay(msg):
    global message_to_tell
    global have_to_talk
    message_to_tell = str(msg.data)
    have_to_talk = True

def onReceiveSayLong(msg):
    global long_message_to_tell
    global have_to_talk_long
    message = str(msg.data)
    long_message_to_tell = message.split(".")
    have_to_talk_long = True

def onReceivePoint(msg):
    global point_screen
    message_to_tell = "So I'm choosing "+ msg.data
    point_screen = True

def onExit(msg):
    global stop
    stop = True


############################################# main loop
if __name__=="__main__":

    rospy.init_node("nao_actions")

    sg.StiffnessOn(motionProxy)
    #sg.telling_arms_gesturs(motionProxy,tts,speed,"hello")

    listener = tf.TransformListener()
    listener.waitForTransform('/base_footprint','/face_0', rospy.Time(0), rospy.Duration(4.0))

    while not stop:

        test  = listener.getFrameStrings()

        rospy.Subscriber('robot_target_topic', String, onReceiveTarget)
        rospy.Subscriber('robot_say_topic', String, onReceiveSay)
        rospy.Subscriber('robot_say_long_topic', String, onReceiveSayLong)
        rospy.Subscriber('robot_point_topic', String, onReceivePoint)
        rospy.Subscriber('exit_topic', String, onExit)

        if point_screen:
            time.sleep(1)
            sg.pointing_object(motionProxy,tts,speed,message_to_tell )
            point_screen = False

        if have_to_talk:
            tracker.stopTracker()
            time.sleep(1)
            sg.telling_arms_gesturs(motionProxy,tts,speed,message_to_tell)
            have_to_talk = False

        if have_to_talk_long:
                # Add target to track.
            targetName = "Face"
            faceWidth = 1#faceSize
            tracker.registerTarget(targetName, faceWidth)
            # Then, start tracker.
            tracker.track(targetName)
            for message in long_message_to_tell:
                time.sleep(1)
                story_telling(message)
                sg.telling_arms_gesturs(motionProxy,tts,speed,message)
            have_to_talk_long = False
            tracker.stopTracker()

        if "base_footprint" in test and "robot_head" in test and "face_0" in test:
            rospy.loginfo("frames found! (child head condition")

            (pose,rot) = listener.lookupTransform('/robot_head','/face_0', rospy.Time(0))

            x = pose[0]
            y = pose[1]
            z = pose[2]

            Zyaw = np.arctan(y/x)
            Zpitch = np.arctan(-z/x)

            euler = tf.transformations.euler_from_quaternion(rot)
            Xroll = euler[1]
            Xpitch = euler[0] - np.pi/2.
            Xyaw = -np.sign(euler[2])*(np.abs(euler[2])-np.pi/2.)

            rospy.loginfo("pitch "+str(Xpitch))
            rospy.loginfo("yaw "+str(Xyaw))

            pitch = Zpitch + Xpitch
            yaw = Zyaw + Xyaw

            if (np.abs(yaw)>np.pi/6. or pitch> np.pi/7. or pitch< -np.pi/6 or stay) and ok:

                stay = True

                motionProxy.setAngles("HeadYaw", yaw, speed)
                motionProxy.setAngles("HeadPitch", pitch, speed)

                if np.random.rand()>0.95:
                    stay = False
                    ok = False

            else:
                listener.waitForTransform('/robot_head', current_target, rospy.Time(0), rospy.Duration(4.0))
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

                motionProxy.setAngles("HeadYaw", yaw, speed)
                motionProxy.setAngles("HeadPitch", pitch, speed)

                if np.random.rand()>0.95:
                    stay = True
                    ok = True


        rospy.sleep(0.2)

    sg.StiffnessOff(motionProxy)

    rospy.spin()
