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


#########################################"### moving functions
def StiffnessOn(motionProxy):
	pNames = "Body"
	pStiffnessLists = 1.0
	pTimeLists = 1.0
	motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def StiffnessOff(motionProxy):
	speed = 0.1
	motionProxy.setAngles("LShoulderPitch", 1.5, speed)
	motionProxy.setAngles("RShoulderPitch", 1.5, speed)
	time.sleep(2)
	pNames = "Body"
	pStiffnessLists = 0.0
	pTimeLists = 1.0
	motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def trackFace(motionProxy,tracker):
	targetName = "Face"
	faceWidth = 0.1
	tracker.registerTarget(targetName, faceWidth)
	# Then, start tracker.
	motionProxy.setStiffnesses("Head", 1.0)
	tracker.track(targetName)

def telling_arms_gesturs(motionProxy,tts,speed,wordToSay):
	LShoulderPitch = np.random.uniform(-0.5,0.7)#-2.0857 to 2.0857
	LShoulderRoll  = np.random.uniform(0.2,0.8)#-0.3142 to 1.3265
	LElbowYaw  = np.random.uniform(-1,1)#-2.0857 to 2.0857
	LElbowRoll  = np.random.uniform(-1,-0.2)#-1.5446 to -0.0349
	LWristYaw  = -1.6#np.random.uniform(1,1.5)#-1.8238 to 1.8238
	LHand = np.random.choice([0,1])

	# assymetric
	RShoulderPitch = np.random.uniform(-0.5,0.7)#-2.0857 to 2.0857
	RShoulderRoll  = np.random.uniform(-0.8,0.2)#-0.3142 to 1.3265
	RElbowYaw  = np.random.uniform(-1,1)#-2.0857 to 2.0857
	RElbowRoll  = np.random.uniform(0.2,1)#-1.5446 to -0.0349
	RWristYaw  = 1.6#np.random.uniform(1,1.5)#-1.8238 to 1.8238
	RHand = LHand
	
	'''
	# symetric
	RShoulderPitch = LShoulderPitch
	RShoulderRoll  = -LShoulderRoll
	RElbowYaw  = -LElbowYaw
	RElbowRoll  = -LElbowRoll
	RWristYaw  = -LWristYaw
	RHand = LHand
	'''

	motionProxy.setAngles("LShoulderPitch", LShoulderPitch, speed)
	motionProxy.setAngles("RShoulderPitch", RShoulderPitch, speed)
	motionProxy.setAngles("LShoulderRoll", LShoulderRoll, speed)
	motionProxy.setAngles("RShoulderRoll", RShoulderRoll, speed)
	motionProxy.setAngles("LElbowYaw", LElbowYaw, speed)
	motionProxy.setAngles("RElbowYaw", RElbowYaw, speed)
	motionProxy.setAngles("LElbowRoll", LElbowRoll, speed)
	motionProxy.setAngles("RElbowRoll", RElbowRoll, speed)
	motionProxy.setAngles("LWristYaw", LWristYaw, speed)
	motionProxy.setAngles("RWristYaw", RWristYaw, speed)
	motionProxy.setAngles("LHand", LHand, speed)
	motionProxy.setAngles("RHand", RHand, speed)
	tts.say(wordToSay)


