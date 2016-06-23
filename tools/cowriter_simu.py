#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model,agent
import matplotlib.pyplot as plt
import copy

def child(name,all_names):
    percepts = ["convergence","tablet","robot_head","noise","look_tablet","look_child_head","look_noise"]
    actions = ["demo","punish","reward","look_robot_head","look_noise","look_tablet"]
    rewards = [["convergence",1.,1.],["convergence",-1.,-1.],["noise",1.,0.5]]
    # "intrinsic" reward for writing_convergence will dicrease with time, the robot has to find a way to incrise the extrinsically (via temporall diff)
    teacher = agent.Agent(name,all_names,percepts,actions,rewards)
    return teacher

def robot(name,all_names):
    percepts = ["child_progress","tablet","child_head","justified_reward","justified_punish","look_tablet","look_child_head","look_noise"]
    actions = ["converge","diverge","exaggerate","look_tablet","look_child_head","look_noise","look_tablet","imitate","point_tablet","new_move"]
    rewards = [["justified_reward",1.,1.],["justified_punish",1.,1],["with_me",1.,1.],["with_me",-1.,-1.],["child_progress",1.,1.]]
    learner = agent.Agent(name,all_names,percepts,actions,rewards)
    return learner

"""
basic simulation of CoWriter interaction
========================================
convergence = the writing of the robot approaches the writing of the child
exaggerate = exaggerate the mistakes of the child
justified_reward/punish = the child gives a reward/punish to the robot while the robot is converging/diverging
new_move = do something new and unexpected
with_me = if the child look at what he is expected to look at (with_me_ness)
"""

# parameters

child_name = "child"
robot_name = "robot"
all_names = [child,robot]

robot_activity_actions = ["converge","diverge","exaggerate"]
robot_interaction_actions = ["look_tablet","look_child_head","look_noise","look_tablet","imitate","point_tablet","new_move"]

child_activity_actions = ["demo"]
child_interaction_actions = ["punish","reward","look_robot_head","look_noise","look_tablet"]

ROBOT_TURN = True # False meaning the child's turn

CHILD_TABLET = False
CHILD_IMPROVE = False
CHILD_ROBOT_HEAD = False
CHILD_WILL_ROBOT = False
CHILD_WILL_TABLET = False

N = 1
n = 20
CUMREW1 = np.zeros(n)
CUMREW2 = np.zeros(n)

def init_percepts():
    # perceived signals
    robot_obs = {}
    robot_obs.setdefault(child_name,[])
    robot_obs.setdefault(robot_name,[])
    robot_obs.setdefault(child_name+':'+robot_name,[])
    child_obs = {}
    child_obs.setdefault(child_name,[])
    child_obs.setdefault(robot_name,[])
    child_obs.setdefault(robot_name+':'+child_name,[])
    # perceived actions
    robot_actions = {}
    robot_actions.setdefault(child_name,[])
    robot_actions.setdefault(robot_name,[])
    robot_actions.setdefault(child_name+':'+robot_name,[])
    child_actions = {}
    child_actions.setdefault(child_name,[])
    child_actions.setdefault(robot_name,[])
    child_actions.setdefault(robot_name+':'+child_name,[])
    return robot_obs,child_obs,robot_actions,child_actions



def activity_update(robot,child,action_child,action_robot,previous):
    global CHILD_IMPROVE
    global CHILD_TABLET
    global CHILD_ROBOT_HEAD
    global CHILD_NOISE
    global CHILD_WILL_ROBOT
    global CHILD_WILL_TABLET

    robot_obs,child_obs,robot_actions,child_actions = init_percepts()

    # robot's turn:
    if action_robot in robot_activity_actions:
        # if the robot think the child is still looking at the tablet:
        if robot.M[child_name].intensities["look_tablet"]>0:
            robot_actions[child_name+":"+robot_name] = action_robot
            if action_robot == "converge":
                robot_obs[child_name].append(("convergence",1.))
            if action_robot == "diverge":
                robot_obs[child_name].append(("convergence",-1.))
            if action_robot == "exaggerate":
                robot_obs[child_name].append(("convergence",-0.5.))

        # actually the child...
        if (CHILD_TABLET and np.random.rand()>0.3) or (CHILD_ROBOT_HEAD and np.random.rand()<0.2):
            child_actions[robot_name] = action_robot
            if action_robot == "converge":
                child_obs[child_name].append(("convergence",1.))
            if action_robot == "diverge":
                child_obs[child_name].append(("convergence",-1.))
            if action_robot == "exaggerate":
                child_obs[child_name].append(("convergence",-0.5.))
                if np.random.rand()>0.3:
                    CHILD_IMPROVE = True

        # big assumption: robot always see child action
        robot_actions[child_name] = action_child
        robot_obs[child_name].append((action_child,1.)

        # if child feedback
        if action_child=="reward":
            if robot.M[child_name].intensities["convergence"]>0:
                robot_obs[robot_name].append(("justified_reward",1.))
        if action_child=="punish":
            if robot.M[child_name].intensities["convergence"]<0:
                robot_obs[robot_name].append(("justified_punish",1.))

        # other assumption : the child is aware of where the robot is looking

        # if robot is lookink the tablet:
        if robot.M[robot_name].intensities["look_tablet"]>0:
            if action_child in ["punish","reward"]:
                # child think the robot sees his action:
                child_actions[robot_name+":"+child_name] = action_child
                # robot think the child think it is rewarded:
                robot_obs[child_name+":"+robot_name].append((action_child,1.))

        # if robot is looking the child:
        if robot.M[robot_name].intensities["look_child_head"]>0:
            if action_child in ["look_robot_head","look_tablet","look_noise"]:
                # child think the robot sees his action:
                child_actions[robot_name+":"+child_name] = action_child
                # it is a posture-action => other postures are false:
                for other_action in set(["look_robot_head","look_tablet","look_noise"])-set([action_child]):
                    robot_obs[child_name].append((other_action,-1.)
            CHILD_ROBOT_HEAD = (action_child=="look_robot_head")
            CHILD_TABLET =  (action_child=="look_tablet")

    # child's turn:
    if action_child in child_activity_actions:

        # big assumption: robot always see child action
        robot_actions[child_name] = action_child
        robot_obs[child_name].append((action_child,1.)

        if CHILD_IMPROVE:
            robot_obs[robot_name].append(("child_progress",1.))

        # if the robot think the child think the robot sees the word:
        if robot.M[robot_name].intensities["look_tablet"]>0:
            robot_obs[child_name+":"+robot_name].append((action_child,1.))
            child_actions[robot_name+":"+child_name] = action_child

        # ["look_tablet","look_child_head","look_noise","point_tablet","new_move"]

        # robot action
        if action_robot=="point_tablet":
            if robot.M[child_name].intensities["look_robot_head"]:
                robot_actions[child_name+":"+robot_name] = action_robot
            if CHILD_ROBOT_HEAD or np.random.rand()>0.3:
                child_actions[robot_name] = action_robot
                CHILD_WILL_TABLET = True


        if action_robot in ["look_tablet","look_child_head","look_noise"]:
            if robot.M[child_name].intensities["look_robot_head"]>0:
                #
                robot_actions[child_name+":"+robot_name] = action_robot
                # it is a posture-action => other postures are false:
                for other_action in set(["look_robot_head","look_tablet","look_noise"])-set([action_robot]):
                    child_obs[robot_name].append((other_action,-1.)
            if CHILD_ROBOT_HEAD:
                #
                child_actions[robot_name] = action_robot
                child_obs[robot_name].append((action_robot,1.))
                # it is a posture-action => other postures are false:
                for other_action in set(["look_robot_head","look_tablet","look_noise"])-set([action_child]):
                    child_obs[robot_name].append((other_action,-1.)

        if action_robot=="new_move":
            robot_actions[child_name+":"+robot_name] = action_robot
            if np.random.rand()>0.3:
                CHILD_WILL_ROBOT = True


    return robot_obs,child_obs,robot_actions,child_actions



case = "MM1"
for i in range(N):
    if i>N/2.:
        case="MM2"
    if i%10==0:
        print i
    teacher = create_teacher(name1,all_names)
    learner = create_learner(name2,all_names)
    cumrew = []
    model_percepts1 = None
    model_percepts2 = None
    model_actions1 = None
    model_actions2 = None
    action1 = ""
    action2 = ""
    previous = []
    for j in range(n):

        action1 = teacher.update_models(None,model_percepts1,model_actions1,case)
        action2 = learner.update_models(None,model_percepts2,model_actions2,case)
        model_percepts1,model_percepts2,model_actions1,model_actions2,r = world_update(action1,action2,previous)
        cumrew.append(r)

    if i>100:
        CUMREW+=(np.arange(n) - np.cumsum(np.array(cumrew)))/float(N)
    else:
        CUMREW2+=(np.arange(n) - np.cumsum(np.array(cumrew)))/float(N)

print 'teacher think about learner:'
teacher.show_learned_rewards('learner')
print ' learner think about teacher:learner '
learner.show_learned_rewards('teacher;learner')
print 'actual learner'
learner.show_learned_rewards('learner')
print "================================="

print ' learner think about teacher:'
learner.show_learned_rewards('teacher')
print ' teacher think about learner:teacher '
teacher.show_learned_rewards('learner;teacher')
print 'actual teacher:'
teacher.show_learned_rewards('teacher')

teacher.show_social_error('learner')

curve1 = teacher.social_curve
curve2 = learner.social_curve

#plt.plot(curve1)
#plt.plot(curve2)
plt.plot(CUMREW)
plt.plot(CUMREW2)
plt.show()
