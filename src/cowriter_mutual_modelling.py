#!/usr/bin/env python
#coding: utf-8

import sys
import time
import rospy
import json
from std_msgs.msg import String, Empty, Float64
from mutualModelling.agent import Agent

# this node udate models of agents (by the robot) and publishes the choice of action by the robot:
#-------------------------------------------------------------------------------------------------
pub_robot_action = rospy.Publisher('robot_action_topic', Float64, queue_size=1)

# create a mutual modeller agent "robot" that also model an agent "human" in cowriter:
#-------------------------------------------------------------------------------------
ROBOT_NAME = "Mimi"
HUMAN_NAME = "Child"
ALL_NAMES = [ROBOT_NAME, HUMANE_NAME]
robot_percepts = ["child_progress","child_head","reward","punish","justified_reward","justified_punish","justified_new_word","with_me"]
robot_actions = ["converges","diverges","exaggerates","looks_tablet","looks_child_head","looks_out","looks_experimentator","looks_selection_tablet","points_tablet"]
robot_rewards = [["justified_reward",1.,1.],["justified_punish",1.,1],["with_me",1.,1.],["with_me",-1.,-1.],["child_progress",1.,1.],["justified_new_word",1.,1.]]
robot = agent.Agent(ROBOT_NAME,ALL_NAMES,robot_percepts,robot_actions,robot_rewards)

# the point of attention of the human is used to define what action of the robot is observed by the child:
#---------------------------------------------------------------------------------------------------------
objects = {"experimentator","selection_tablet","tablet","robot_head","out"}
human_attention = ""

# what the human can perceive about robot actions given his point of attention:
visible_for_human_from = {"tablet":["converges","diverges"], "robot_head":["looks_tablet","looks_child_head","looks_out","points_tablet","looks_experimentator"]}

# what the robot is expected to perceive about human action given robot's attention:
# (the robot is not expected (by the child) to differentiate justified/unjustified behavior of the child)
visible_for_robot_from = {"tablet":["punishes","rewards","sends_demo"],"selection_tablet":["sends_new_word"], "child_head":["looks_tablet","looks_robot_head","looks_out","looks_experimentator"]}

# when an agent do/observe something the mutual models (by the robot) are updated:
#---------------------------------------------------------------------------------
def onRobotAction(msg):


if __name__=='name':
    rospy.Suscriber('robot_action_topic', String, onRobotAction )
    rospy.Suscriber('human_action_topic', String, onHumanAction)
    rospy.Suscriber('robot_target_topic', String, onChangeRobotTarget)
    rospy.Suscriber('human_target_topic', String, onChangeHumanTarget)
    rospy.Suscriber('robot_obs_topic', String, onRobotObs)
    rospy.Suscriber('human_obs_topic', String, onHumanObs)
