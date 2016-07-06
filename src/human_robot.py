#!/usr/bin/env python
#coding: utf-8

import sys
import time
import rospy
import json
from std_msgs.msg import String, Empty, Float64
from mutualModelling.agent import Agent

# this node publishes the choice of action by the robot:
#-------------------------------------------------------
pub_robot_action = rospy.Publisher('robot_action_topic', Float64, queue_size=1)

# create a mutual modeller agent "robot" that also model an agent "human":
#-------------------------------------------------------------------------
robot_percepts = ["child_progress","child_head","justified_reward","justified_punish"]
robot_actions = ["converge","diverge","exaggerate","look_tablet","look_child_head","look_out","look_tablet","point_tablet"]
robot_rewards = [["justified_reward",1.,1.],["justified_punish",1.,1],["with_me",1.,1.],["with_me",-1.,-1.],["child_progress",1.,1.]]
robot = agent.Agent("robot",["robot","human"],robot_percepts,robot_actions,robot_rewards)

# the point of attention of the human is used to define what action of the robot is observed by the child:
#---------------------------------------------------------------------------------------------------------
objects = {"experimentator","selection_tablet","tablet","robot_head","outside"}
human_attention = ""

def humanAttention(object):

def onReceiveInfoHuman(msg):

def onReceiveInfoRobot(msg):

if __name__='name':
