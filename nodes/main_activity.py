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
from nextChoice import story_maker as sm
from nextChoice.decision import decision_maker as dm

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

########################################## ros publishers
# for nao_actions:
pub_robot_target = rospy.Publisher('robot_target_topic', String, queue_size=1)
pub_robot_say = rospy.Publisher('robot_say_topic', String, queue_size=1)
pub_robot_point = rospy.Publisher('robot_point_topic', String, queue_size=1)
pub_exit = rospy.Publisher('exit_topic', String, queue_size=1)

# for interface:
pub_human_turn = rospy.Publisher('human_turn_topic', String, queue_size=1)
pub_human_chosen = rospy.Publisher('human_chosen_topic', String, queue_size=1)
pub_human_predict = rospy.Publisher('human_predict_topic', String, queue_size=1)
pub_robot_turn = rospy.Publisher('robot_turn_topic', String, queue_size=1)
pub_robot_chosen = rospy.Publisher('robot_chosen_topic', String, queue_size=1)
########################################## action robot functions

def look_at(target):
	msg = String()
	msg.data = target
	pub_robot_target.publish(msg)

def say(to_say):
	msg = String()
	msg.data = to_say
	pub_robot_say.publish(msg)

def point(button):
	msg = String()
	msg.data = button
	pub_robot_point.publish(msg)

def ending(test):
	msg = String()
	msg.data = ""
	pub_exit.publish(msg)


if __name__=="__main__":

	rospy.init_node("main_activity")

	time.sleep(10)
	say("hello, my name is Nando.")
	time.sleep(5)
	say("do you want to write an amazing story with me ?")
	time.sleep(10)
	look_at("experimentator")
	time.sleep(5)
	look_at("child_head")
	time.sleep(5)

	ending("")


	
