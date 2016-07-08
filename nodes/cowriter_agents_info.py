#!/usr/bin/env python
#coding: utf-8

import sys
import time
import rospy
import json
from std_msgs.msg import String, Empty, Float64
from mutualModelling.agent import Agent

# this node collect all information about agents states/obs/actions:
# and publish in one topic per agent (human, robot)
#-------------------------------------------------------------------------
pub_robot_action = rospy.Publisher('robot_action_topic', String, queue_size=1)
pub_human_action = rospy.Publisher('human_action_topic', String, queue_size=1)
pub_robot_target = rospy.Publisher('robot_target_topic', String, queue_size=1)
pub_human_target = rospy.Publisher('human_target_topic', String, queue_size=1)
pub_robot_obs = rospy.Publisher('robot_obs_topic', String, queue_size=1)
pub_human_obs = rospy.Publisher('human_obs_topic', String, queue_size=1)


def onChangeHumanTarget(msg):
    current_human_target = str(msg.data)#.split("_/")[1]
    target = String()
    target.data = current_human_target
    pub_human_target.publish(target)
    # it also defines an action of the human:
    human_action = "looks_"+current_human_target
    action = String()
    action.data = human_action
    pub_human_action.publish(action)

"""
def onHumanAction(msg):
    pub_human_action.publish(msg)

def onRobotAction(msg):
    pub_robot_action.publish(msg)
    robot_action = str(msg.data)
    if "looks" == robot_action.split("_")[0] and len(robot_action.split("_"))>0:
        # then it also defines a new visual target of the robot:
        current_robot_target = '_'.join(robot_action.split("_")[1:])
        target = String()
        target.data = current_human_target
        pub_robot_target.publish(target)
"""
# TODO:
# def onRobotObs(msg):
#
# def onHumanObs(msg):

if __name__=='__main__':

    rospy.init_node("cowriter_agent_info")

    while(True):
        # TODO: get this info from cowriter nodes
        # instead of a tool_simu:
        #
        # # get current task:
        # rospy.Subscriber("state_activity", String, onChangeTask)
        #
        # # get human activity action
        # rospy.Suscriber("")
        #
        # get robot action:
        #
        # get human action:

        # get current human target: (~attention_tracker)
        rospy.Subscriber("actual_focus_of_attention", String, onChangeHumanTarget)

        # get human action: (~ tool~simu)
        #rospy.Subscriber("human_action_topic", String, onHumanAction)

        # get robot action: (~ tool_simu)
        #rospy.Subscriber("robot_action_topic", String, onRobotAction)

        # TODO: same with obs (not from agents actions but from the world)

        rospy.sleep(1.0)

    rospy.spin()
