#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model,agent
import matplotlib.pyplot as plt
import copy

def child(name,all_names):
    percepts = ["convergence","tablet","robot_head","noise"]
    actions = ["demo","punish","reward","look_robot_head","look_noise","look_tablet"]
    rewards = [["convergence",1.,1.],["convergence",-1.,-1.],["noise",1.,0.5]]
    # "intrinsic" reward for writing_convergence will dicrease with time, the robot has to find a way to incrise the extrinsically (via temporall diff)
    teacher = agent.Agent(name,all_names,percepts,actions,rewards)
    return teacher

def robot(name,all_names):
    percepts = ["robot_progress","child_progress","writing_convergence","tablet","child_head","justified_reward","justified_punish"]
    actions = ["converge","diverge","exaggerate","look_tablet","look_child_head","look_noise","look_tablet","imitate","point_tablet","new_move"]
    rewards = [["justified_reward",1.,1.],["justified_punish",1.,1],["with_me",1.,1.],["with_me",-1.,-1.]]
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

robot_turn = True # False meaning the child's turn

N = 1
n = 20
CUMREW1 = np.zeros(n)
CUMREW2 = np.zeros(n)

def world_update(action1,action2,previous):
    # robot's turn:

    # child's turn:

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
