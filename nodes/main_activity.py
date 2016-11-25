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
from NaoStoryTelling import story_gestures as sg

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

############################################# global values
NAO_IP = "146.193.224.10"
port = 9559
speed = 0.1

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

if __name__=="__main__":

	sg.StiffnessOn(motionProxy)


	sg.trackFace(motionProxy,tracker)

	for i in range(1,10):

		time.sleep(1)
		sg.telling_arms_gesturs(motionProxy,tts,speed,str(2**i))
	

	sg.StiffnessOff(motionProxy)
