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
from nextChoice.decision import *

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
pub_human_predict = rospy.Publisher('human_predict_turn_topic', String, queue_size=1)
pub_robot_turn = rospy.Publisher('robot_turn_topic', String, queue_size=1)
pub_robot_chosen = rospy.Publisher('robot_chosen_topic', String, queue_size=1)
########################################## publishing

def look_at(target):
	msg = String()
	msg.data = target
	pub_robot_target.publish(msg)

def say(to_say):
	msg = String()
	msg.data = to_say
	pub_robot_say.publish(msg)

def point(robot_choice):
	msg = String()
	msg.data = robot_choice
	pub_robot_point.publish(msg)

def ending(test):
	msg = String()
	msg.data = ""
	pub_exit.publish(msg)

def robot_turn(label, items):
	msg = String()
	msg.data = label+",_,"+"-".join(items)
	pub_robot_turn.publish(msg)

def human_turn(label, items):
	msg = String()
	msg.data = label+",_,"+"-".join(items)
	pub_human_turn.publish(msg)

def human_predict(label, items):
	msg = String()
	msg.data = label+",_,"+"-".join(items)
	pub_human_predict.publish(msg)

def human_chosen(label, items, choice):
	msg = String()
	msg.data = label+","+choice+","+"-".join(items)
	pub_human_chosen.publish(msg)

def robot_chosen(label, items, choice):
	msg = String()
	msg.data = label+","+choice+","+"-".join(items)
	pub_robot_chosen.publish(msg)

######################################## reacting
last_human_prediction = ""
last_human_choice = ""
chosen = False
received = False
choice = ""

def onNewChoice(msg):
	global chosen
	global choice
	global last_human_choice
	last_human_choice = msg.data
	chosen = True
	choice = msg.data

def onNewPrediction(msg):
	global chosen
	global choice
	global last_human_prediction
	last_human_prediction = msg.data
	chosen = True
	choice = msg.data

def onReceived(msg):
	global received
	received = True



sequence = [("main character is ...", sm.C_MCg), ()]

robot = decision_maker("incoherant")


if __name__=="__main__":

	rospy.init_node("main_activity")

	rospy.sleep(5)
	say("hello, my name is Nando.")
	rospy.sleep(5)
	say("do you want to write an amazing story with me ?")
	rospy.sleep(5)
	look_at("child_head")
	rospy.sleep(1)

	say("OK ! first, we need a main character !")
	rospy.sleep(5)
	say("What do you prefere, a man, a woman or a robot ?")
	rospy.sleep(1)


	human_turn("main character is ...", sm.C_MCg)

	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("main character is ...", sm.C_MCg, choice)
	rospy.sleep(5)

	human_predict("predict main character job is ...", sm.C_MCj_man)

	while not chosen:
		rospy.Subscriber('human_prediction_topic', String, onNewPrediction)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("predict main character job is ...", sm.C_MCj_man, choice)
	rospy.sleep(1)

	robot_turn("main character drink is ...", sm.C_MCj_man)
	rospy.sleep(5)

	robot_choice = robot.choose(last_human_choice, last_human_prediction, sm.C_MCj_man)
	point(robot_choice)
	rospy.sleep(1)

	robot_chosen("main character drink is ...", sm.C_MCj_man, robot_choice)
	rospy.sleep(5)



	ending("")


	
