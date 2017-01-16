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
from nextChoice.story_maker import story
from nextChoice.decision import *

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

########################################## ros publishers
# for nao_actions:
pub_robot_target = rospy.Publisher('robot_target_topic', String, queue_size=1)
pub_robot_say = rospy.Publisher('robot_say_topic', String, queue_size=1)
pub_robot_say_long = rospy.Publisher('robot_say_long_topic', String, queue_size=1)
pub_robot_point = rospy.Publisher('robot_point_topic', String, queue_size=1)
pub_exit = rospy.Publisher('exit_topic', String, queue_size=1)

# for interface:
pub_human_turn = rospy.Publisher('human_turn_topic', String, queue_size=1)
pub_human_chosen = rospy.Publisher('human_chosen_topic', String, queue_size=1)
pub_human_predict = rospy.Publisher('human_predict_turn_topic', String, queue_size=1)
pub_robot_turn = rospy.Publisher('robot_turn_topic', String, queue_size=1)
pub_robot_chosen = rospy.Publisher('robot_chosen_topic', String, queue_size=1)
pub_new_element = rospy.Publisher('new_element', String, queue_size=1)

########################################## publishing

def look_at(target):
	msg = String()
	msg.data = target
	pub_robot_target.publish(msg)

def say(to_say):
	msg = String()
	msg.data = to_say
	pub_robot_say.publish(msg)

def say_long(to_say):
	msg = String()
	msg.data = to_say
	pub_robot_say_long.publish(msg)

def point(robot_choice):
	msg = String()
	msg.data = robot_choice; rospy.loginfo("robot_choice "+robot_choice)
	pub_robot_point.publish(msg)

def ending(test):
	msg = String()
	msg.data = ""
	pub_exit.publish(msg)

def robot_turn(label, items, name="false"):
	msg = String()
	msg.data = label+",_,"+"-".join(items)+','+name
	pub_robot_turn.publish(msg)

def human_turn(label, items, name="false"):
	msg = String()
	msg.data = label+",_,"+"-".join(items)+','+name
	pub_human_turn.publish(msg)

def human_predict(label, items, name="false"):
	msg = String()
	msg.data = label+",_,"+"-".join(items)+','+name
	pub_human_predict.publish(msg)

def human_chosen(label, items, choice, name="false"):
	msg1 = String()
	msg2 = String()
	msg1.data = label+","+choice+","+"-".join(items)+','+name
	pub_human_chosen.publish(msg1)
	msg2.data = "(You) "+label+"\n"
	pub_new_element.publish(msg2)

def robot_chosen(label, items, choice, name="false"):
	msg1 = String()
	msg2 = String()
	msg1.data = label+","+choice+","+"-".join(items)+','+name
	pub_robot_chosen.publish(msg1)
	msg2.data = "(Nando) "+label+"\n"
	pub_new_element.publish(msg2)

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

robot = decision_maker("coherant")


if __name__=="__main__":

	rospy.init_node("main_activity")

	story = story()
	result = story.generate()
	rospy.loginfo(result)

	###################################

	rospy.sleep(6)
	say("hello, my name is Nando.")
	rospy.sleep(4)

	say("do you want to write an amazing story with me ?")
	rospy.sleep(3)

	look_at("child_head")
	rospy.sleep(1)

	say("OK ! first, we need to choose where the story takes place ! What do you prefere ?")
	rospy.sleep(6)

	########################################## PLACE

	human_turn("What is the place of the story... ", sm.C_P)

	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	story.P = choice
	story.update()
	human_chosen("The story takes place in a "+story.P, sm.C_P, choice)

	say("in a"+story.P+" !, well!")
	rospy.sleep(4)

	look_at("child_head")
	rospy.sleep(1)

	say("Now it's my turn, I need to choose the peacefull people of the place. But first, I would like to know what do you think I am going to choose.")
	rospy.sleep(7)

	################################### PEOPLE

	# prediction

	human_predict("predict what the robot is going to choose ...", sm.C_Pp)

	while not chosen:
		rospy.Subscriber('human_prediction_topic', String, onNewPrediction)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("you predicted "+choice, sm.C_Pp, choice)
	rospy.sleep(3)

	# actual

	robot_choice = robot.choose(last_human_choice, last_human_prediction, sm.C_Pp)
	story.Pp = robot_choice; rospy.loginfo("robot_choice "+robot_choice)
	story.update()

	say(choice+" is a great idea...")
	rospy.sleep(2)

	robot_turn("peacefull peaople of the place are ...", sm.C_Pp)
	rospy.sleep(3)

	point(robot_choice)
	go_in = True
	if go_in:
		if robot_choice!=choice:
			say("but I am going for "+story.Pp+"!")
			go_in = False
		else:
			say("That's what I choose, "+story.Pp+"... well done !")
			go_in = False
	rospy.sleep(2)

	robot_chosen("peacefull peaople of the place are "+robot_choice, sm.C_Pp, robot_choice)

	say("Ok your turn! we need a main character! what do you prefer, a man, a woman or a robot ?")
	rospy.sleep(6)

	################################### MC GENDER

	human_turn("main character is ...", sm.C_MCg)

	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	story.MCg = choice
	story.update()
	human_chosen("main character is a "+story.MCg+" !", sm.C_MCg, choice)
	say("a "+story.MCg+" ! Nice ! and what would be "+story.MC_ppos+" name ?")
	rospy.sleep(5)

	################################### MC NAME

	human_turn("The name of the main character is ...", sm.C_MC, "true")

	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	story.MC = choice
	story.update()
	human_chosen(story.MC_ppos+" name is "+story.MC+" !", sm.C_MC, choice, "true")

	say(story.MC+" ! Hmm, that's a good name !")
	rospy.sleep(4)

	look_at("child_head")
	rospy.sleep(1)

	say("And it's my turn, we need to find the main character's job. What do you predict I will choose ?")
	rospy.sleep(6)

	################################### MC JOB

	# prediction

	human_predict("predict what the robot is going to choose ...", story.C_MCj)

	while not chosen:
		rospy.Subscriber('human_prediction_topic', String, onNewPrediction)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("you predicted "+choice, story.C_MCj, choice)
	rospy.sleep(3)

	# actual

	robot_choice = robot.choose(last_human_choice, last_human_prediction, story.C_MCj)
	story.MCj = robot_choice; rospy.loginfo("robot_choice "+robot_choice)
	story.update()

	say(choice+" is a great idea...")
	rospy.sleep(2)

	robot_turn("main character job is ... ", story.C_MCj)
	rospy.sleep(3)

	point(robot_choice)
	go_in = True
	if go_in:
		if robot_choice!=choice:
			say("but I am going for "+story.MCj+"!")
			go_in = False
		else:
			say("That's what I choose, "+story.MCj+"... well done !")
			go_in = False
	rospy.sleep(2)

	robot_chosen("main character job is "+robot_choice, story.C_MCj, robot_choice)

	say("Ok your turn! let's find a favorite drink for the main character!")
	rospy.sleep(6)

	################################### MC DRINK

	human_turn("Main character favourite drink ...", sm.C_MCd)

	chosen = False
	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	story.MCd = choice
	story.update()
	human_chosen(story.MC_ppos+" favourite drink is "+story.MCd+" !", sm.C_MCd, choice)

	say(story.MC_pper_s+" likes "+story.MCd+" ,ok!")
	rospy.sleep(4)

	look_at("child_head")
	rospy.sleep(1)

	say("Now it's me, I have to find his weapon ! but again, what du you predict?")
	rospy.sleep(6)

	################################## MC WEAPON

	# prediction

	human_predict("predict what the robot is going to choose ...", sm.C_MCw)

	while not chosen:
		rospy.Subscriber('human_prediction_topic', String, onNewPrediction)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("you predicted "+choice, sm.C_MCw, choice)
	rospy.sleep(3)

	# actual

	robot_choice = robot.choose(last_human_choice, last_human_prediction, sm.C_MCw)
	story.MCw = robot_choice; rospy.loginfo("robot_choice "+robot_choice)
	story.update()

	say(choice+", interesting...")
	rospy.sleep(2)

	robot_turn("main character weapon is ...", sm.C_MCw)
	rospy.sleep(3)

	point(robot_choice)
	go_in = True
	if go_in:
		if robot_choice!=choice:
			say("but I am choosing "+story.MCw+"!")
			go_in = False
		else:
			say("Ok for "+story.MCw+", I agree with you !")
			go_in = False
	rospy.sleep(2)

	robot_chosen("main character weapon is "+robot_choice, sm.C_MCw, robot_choice)

	say("Your turn. What is the favourite dance of the main character ?")
	rospy.sleep(6)

	#################################### # main char danse (HACK)

	human_turn("What is the favourite danse of the main character ?", sm.C_SC_dance)

	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	#story.P = choice
	#story.update()
	human_chosen(story.MC_pper_s+" dances "+choice, sm.C_SC_dance, choice)

	say(story.MC_pper_s+" dances "+choice+" !, well!")
	rospy.sleep(4)

	look_at("child_head")
	rospy.sleep(1)

	say("And it's my turn, we need a second character. What do you predict I will choose ?")
	rospy.sleep(6)

	############################################## SC GENDER

	# prediction

	human_predict("predict what the robot is going to choose ...", sm.C_SCg)

	while not chosen:
		rospy.Subscriber('human_prediction_topic', String, onNewPrediction)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("you predicted "+choice, sm.C_SCg, choice)
	rospy.sleep(3)

	# actual

	robot_choice = robot.choose(last_human_choice, last_human_prediction, sm.C_SCg)
	story.SCg= robot_choice; rospy.loginfo("robot_choice "+robot_choice)
	story.update()

	say("it could be a "+choice+"...")
	rospy.sleep(2)

	robot_turn("Second character is ...", sm.C_SCg)
	rospy.sleep(3)

	point(robot_choice)
	go_in = True
	if go_in:
		if robot_choice!=choice:
			say("but I prefere a "+story.SCg+"!")
			go_in = False
		else:
			say("Ok this is what I choose !")
			go_in = False
	rospy.sleep(2)

	robot_chosen("The second character is a "+story.SCg, sm.C_SCg, robot_choice)

	say("I also have to find a name ! What do you predict ?")
	rospy.sleep(5)

	############################################## SC name

	# prediction

	human_predict("predict what the robot is going to choose ...", sm.C_SC, "true")

	while not chosen:
		rospy.Subscriber('human_prediction_topic', String, onNewPrediction)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("you predicted "+choice, sm.C_SC, choice, "true")
	rospy.sleep(3)

	# actual

	robot_choice = robot.choose(last_human_choice, last_human_prediction, sm.C_SC)
	story.SC= robot_choice; rospy.loginfo("robot_choice "+robot_choice)
	story.update()

	say(choice+" could work for "+story.SC_pper_o+"...")
	rospy.sleep(2)

	robot_turn("Second character name is ...", sm.C_SC, "true")
	rospy.sleep(3)

	point(robot_choice)
	go_in = True
	if go_in:
		if robot_choice!=choice:
			say("but "+story.SC_ppos+" name will be "+story.SC+"!")
			go_in = False
		else:
			say("You predicted well, I go for "+story.SC+"!")
			go_in = False
	rospy.sleep(2)

	robot_chosen("The second character is called "+story.SC, sm.C_SC, robot_choice, "true")

	say("Well well well. It's your turn, and you have to find what is the second character's favorite dance.")
	rospy.sleep(6)

	############################################## SC dance

	human_turn("Second character dances ... ", sm.C_SC_dance)

	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	story.SC_dance = choice
	story.update()
	human_chosen("Second character dances "+story.SC_dance, sm.C_SC_dance, choice)

	say(story.SC_pper_s+" dances "+story.SC_dance+", ok.")
	rospy.sleep(4)

	look_at("child_head")
	rospy.sleep(1)

	say("Now, we have to choose the specy of the second character. And it's my turn ! what do you predict ?")
	rospy.sleep(6)

	############################################## SC specie

	# prediction

	human_predict("predict what the robot is going to choose ...", story.C_SCs)

	chosen=False
	while not chosen:
		rospy.Subscriber('human_prediction_topic', String, onNewPrediction)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("you predicted "+choice, story.C_SCs, choice)
	rospy.sleep(3)

	# actual

	robot_choice = robot.choose(last_human_choice, last_human_prediction, story.C_SCs)
	story.SCs= robot_choice; rospy.loginfo("robot_choice "+robot_choice)
	story.update()

	say(choice+" is not a bad idea... ")
	rospy.sleep(2)

	robot_turn("Second character specie is...", story.C_SCs)
	rospy.sleep(3)

	point(robot_choice)
	go_in = True
	if go_in:
		if robot_choice!=choice:
			say("but "+story.SC_pper_s+" will be a "+story.SCs+"!")
			go_in = False
		else:
			say("indeed, I go for "+story.SCs+"!")
			go_in = False
	robot_chosen("The second character is a "+story.SCs, story.C_SCs, robot_choice)
	rospy.sleep(2)

	say("Ok, now it is your turn, and we need to choose a bad guy. Is the bad guy a man, a woman or a robot ?")
	rospy.sleep(6)

	############################################## BG gender

	human_turn("The bad guy is ... ", sm.C_BGg)
	chosen = False

	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	story.BGg = choice
	story.update()
	human_chosen("the bad guy is a "+story.BGg, sm.C_BGg, choice)

	say("the bad guy is a "+story.BGg+", good.")
	rospy.sleep(4)

	look_at("child_head")
	rospy.sleep(1)

	say("and "+story.BG_ppos+" name ?")

	############################################## BG name

	human_turn("Bad guy is named ... ", sm.C_BG, "true")

	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	story.BG = choice
	#story.update()
	human_chosen("Bad guy is named "+story.BG, sm.C_BG, choice, "true")

	say(" bad guy is called "+story.BG+", ok.")
	rospy.sleep(4)

	look_at("child_head")
	rospy.sleep(1)

	say("It's my turn ! I am going to choose the job of the bad guy. What do you predict ?")
	rospy.sleep(6)

	############################################## BG job

	# prediction

	human_predict("predict what the robot is going to choose ...", story.C_BGj)

	while not chosen:
		rospy.Subscriber('human_prediction_topic', String, onNewPrediction)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("you predicted "+choice, story.C_BGj, choice)
	rospy.sleep(3)

	# actual

	robot_choice = robot.choose(last_human_choice, last_human_prediction, story.C_BGj)
	story.BGj= robot_choice; rospy.loginfo("robot_choice "+robot_choice)
	story.update()

	say("well, "+choice+" is also interesting...")
	rospy.sleep(2)

	robot_turn("bad guy's job is ...", story.C_BGj)
	rospy.sleep(3)

	point(robot_choice)
	go_in = True
	if go_in:
		if robot_choice!=choice:
			say("but I go for "+story.BGj+"!")
			go_in = False
		else:
			say("Nice prediction, I'm choosing "+story.BGj+"!")
			go_in = False
	robot_chosen("The bad guy job is "+story.BGj, story.C_BGj, robot_choice)
	rospy.sleep(2)

	say("Your turn ! the bad guy is going to have servant, what kind of servants ?")
	rospy.sleep(6)

	############################################## BG dogs

	human_turn("Bad guy servants are ... ", sm.C_BGd)

	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	story.BGd = choice
	story.update()
	human_chosen("Bad guy servants are "+story.BGd, sm.C_BGd, choice)

	say(" bad guy is served by "+story.BGd+", hmm.")
	rospy.sleep(4)

	look_at("child_head")
	rospy.sleep(1)

	say("Now, I have to figure out what bad actions these servantss are going to perform. what do you predict ?")
	rospy.sleep(6)

	############################################## BG action

	# prediction

	human_predict("predict what the robot is going to choose ...", sm.C_Ba)

	while not chosen:
		rospy.Subscriber('human_prediction_topic', String, onNewPrediction)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("you predicted "+choice, sm.C_Ba, choice)
	rospy.sleep(3)

	# actual

	robot_choice = robot.choose(last_human_choice, last_human_prediction, sm.C_Ba)
	story.Ba= robot_choice; rospy.loginfo("robot_choice "+robot_choice)
	story.update()

	say(choice+", that's bad, I like it...")
	rospy.sleep(2)

	robot_turn("The bad action performed by bad guy's serviters is ...", sm.C_Ba)
	rospy.sleep(3)

	point(robot_choice)
	go_in = True
	if go_in:
		if robot_choice!=choice:
			say("but "+story.Ba+" is even worst !")
			go_in = False
		else:
			say("Off course, I go for "+story.Ba+"!")
			go_in = False
	robot_chosen("The bad action is "+story.Ba, sm.C_Ba, robot_choice)
	rospy.sleep(2)

	say("Great. Now it's your turn. We need to choose the favourite drink of the bad guy!")
	rospy.sleep(6)

	############################################## BG drink

	human_turn("Bad guy's favourite drink ... ", sm.C_BGdrink)

	while not chosen:
		rospy.Subscriber('human_choice_topic', String, onNewChoice)
		rospy.sleep(0.05)

	chosen = False
	story.BGdrink = choice
	story.update()
	human_chosen(" Bad guy's favourite drink is "+story.BGdrink, sm.C_BGdrink, choice)

	say(" bad guy drinks "+story.BGdrink+", well.")
	rospy.sleep(4)

	look_at("child_head")
	rospy.sleep(1)

	say("We have almost everything. I just still have to find the place of the bad guy. What do you predict ?")
	rospy.sleep(6)

	############################################## BG place

	# prediction

	human_predict("predict what the robot is going to choose ...", sm.C_BGp)

	while not chosen:
		rospy.Subscriber('human_prediction_topic', String, onNewPrediction)
		rospy.sleep(0.05)

	chosen = False
	human_chosen("you predicted "+choice, sm.C_BGp, choice)
	rospy.sleep(3)
	story.update()

	# actual

	robot_choice = robot.choose(last_human_choice, last_human_prediction, sm.C_BGp)
	story.BGp= robot_choice; rospy.loginfo("robot_choice "+robot_choice)

	say(choice+" would fit well...")
	rospy.sleep(2)

	robot_turn("The bad guy lives in a ...", sm.C_BGp)
	rospy.sleep(3)

	point(robot_choice)
	go_in = True
	if go_in:
		if robot_choice!=choice:
			say("but "+story.BGp+" is more fun!")
			go_in = False
		else:
			say("That's also my choice, "+story.BGp+"!")
			go_in = False
		robot_chosen("The bad guy lives in a "+story.BGp, sm.C_BGp, robot_choice)
	rospy.sleep(2)


	say("ok now I can guess the story, let me tell it for you.")
	rospy.sleep(4)

	result = story.generate()
	rospy.loginfo(result)

	say_long(result)
	rospy.sleep(10)


	say("Good by, it was a pleasure to work with you !")
	rospy.sleep(4)
	say("ha !")
	rospy.sleep(2)
	ending("")
