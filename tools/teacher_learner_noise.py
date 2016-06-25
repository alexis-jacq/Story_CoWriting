#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model,agent
import matplotlib.pyplot as plt
import copy

def create_teacher(name,all_names):
    percepts = ["a","b","c"]
    actions = ["a","b","c","reward","punish"]
    #rewards = [["success",1.,1.],["fail",1.,-1.]]
    rewards = [["a",1.,-1.],["b",1.,1.],['c',1.,-1.]]
    teacher = agent.Agent(name,all_names,percepts,actions,rewards)
    return teacher

def create_learner(name,all_names):
    percepts = ["reward","noise"]
    actions = ["a","b","c","imitate"]
    rewards = [["reward",1.,1.],["punish",1.,-1],["noise",1.,0.1]]
    learner = agent.Agent(name,all_names,percepts,actions,rewards)
    return learner


"""
"""

# parameters

name1 = "teacher"
name2 = "learner"
all_names = [name1,name2]

N = 20
n = 2000
CUMREW = np.zeros(n)
CUMREW2 = np.zeros(n)
L_curve1 = np.zeros(n-1)
L_curve2 = np.zeros(n-1)
T_curve1 = np.zeros(n-1)
T_curve2 = np.zeros(n-1)

def world_update(action1,action2,previous):
    real_action = action2
    p1 = [(action2,1.)]
    p2 = [(action1,1.)]
    if action2 =="imitate":
        real_action = action1
        p1.append((real_action,1.))
    r = 0
    if "b"==real_action:
        #p1.append(("success",1.))
        #p2.append(("success",1.))
        r = 1
    else:
        #p1.append(("fail",1.))
        #p2.append(("fail",1.))
        if action2=="c":
            p2.append(("noise",1))

    # suppose no errors of perception:

    #model_percepts1 = {name1:p1}#,name2:p2,name2+":"+name1:p1}
    #model_percepts2 = {name2:p2}#,name1:p1,name1+":"+name2:p2}
    #model_percepts1 = {name1:p1,name2:p2}#,name2+":"+name1:p1}
    #model_percepts2 = {name2:p2,name1:p1}#,name1+":"+name2:p2}
    model_percepts1 = {name1:p1,name2:p2,name2+":"+name1:p1}
    model_percepts2 = {name2:p2,name1:p1,name1+":"+name2:p2}
    model_actions1 = {name2:action2,name2+":"+name1:action1}
    model_actions2 = {name1:action1,name1+":"+name2:action2}

    return model_percepts1,model_percepts2,model_actions1,model_actions2,r

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

    if i>N/2.:
        CUMREW+=2*(np.arange(n) - np.cumsum(np.array(cumrew)))/float(N)
        L_curve1 += np.array(learner.social_curve)/float(N)
        T_curve1 += np.array(teacher.social_curve)/float(N)
    else:
        CUMREW2+=2*(np.arange(n) - np.cumsum(np.array(cumrew)))/float(N)
        L_curve2 += np.array(learner.social_curve)/float(N)
        T_curve2 += np.array(teacher.social_curve)/float(N)

print 'teacher think about learner:'
teacher.show_learned_rewards('learner')
print ' learner think about teacher:learner '
learner.show_learned_rewards('teacher:learner')
print 'actual learner'
learner.show_learned_rewards('learner')
print "================================="

print ' learner think about teacher:'
learner.show_learned_rewards('teacher')
print ' teacher think about learner:teacher '
teacher.show_learned_rewards('learner:teacher')
print 'actual teacher:'
teacher.show_learned_rewards('teacher')

teacher.show_social_error('learner')

plt.subplot(3, 1, 1)
plt.plot(CUMREW,'b')
plt.plot(CUMREW2,'r')

plt.subplot(3, 1, 2)
plt.plot(L_curve1,'b')
plt.plot(L_curve2,'r')

plt.subplot(3, 1, 3)
plt.plot(T_curve1,'b')
plt.plot(T_curve2,'r')

plt.show()
