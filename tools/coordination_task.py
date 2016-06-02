#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model,agent
import matplotlib.pyplot as plt
import copy

def create_prisoner(name,all_names):
    percepts = ["reward"]
    actions = ["a","b","c","d","1","2","3","4"]
    rewards = [["reward",1.,1.]]
    prisoner = agent.Agent(name,all_names,percepts,actions,rewards)
    return prisoner

"""
"""

# parameters

name1 = "robert"
name2 = "pierrot"
all_names = [name1,name2]

play = ["1","2","3","4"]

N = 1
n = 500
CUMREW1 = np.zeros(n)
CUMREW2 = np.zeros(n)

def world_update_communicate(action1,action2):
    p1 = []
    p2 = []
    p1.append((name2+"-"+action2,1.))
    p2.append((name1+"-"+action1,1.))
    # suppose no errors of perception:
    model_percepts1 = {name1:p1,name2:p2,name2+";"+name1:p1}
    model_percepts2 = {name2:p2,name1:p1,name1+";"+name2:p2}
    #return model_percepts1,model_percepts2
    return None,None

def world_update_play(action1,action2,previous):
    p1 = []
    p2 = []
    rew = 0.
    if action1==action2 and not action1 in previous:
        rew = 1.
    r1 = rew
    p1.append(("reward",rew))
    r2 = rew
    p2.append(("reward",rew))
    # suppose no errors of perception:
    model_percepts1 = {name1:p1,name2:p2,name2+";"+name1:p1}
    model_percepts2 = {name2:p2,name1:p1,name1+";"+name2:p2}
    return model_percepts1,model_percepts2,r1,r2

for i in range(N):
    if i%10==0:
        print i
    robert = create_prisoner(name1,all_names)
    pierrot = create_prisoner(name2,all_names)
    cumrew1 = []
    cumrew2 = []
    model_percepts1 = None
    model_percepts2 = None
    action1 = ""
    action2 = ""
    previous = []
    for j in range(n):


        if action2 in play:
            action1 = robert.update_models(play,(model_percepts1))
            print action1
            model_percepts1,model_percepts2,r1,r2 = world_update_play(action1,action2,previous)
            cumrew1.append(r1)
            cumrew2.append(r2)
            if action1==action2 and not action1 in previous:
                if len(previous)==len(play)-1:
                    del previous[0]
                previous.append(action1)
            action1 = ""
            action2 = ""
        else:
            action1 = robert.update_models(None,copy.deepcopy(model_percepts1))
            print action1
            if not (action1 in play) and action2:
                model_percepts1,model_percepts2 = world_update_communicate(action1,action2)
                #print "model_percepts2 modified ! !!!!!!"
                #print "================================="
                cumrew1.append(0)
                cumrew2.append(0)
                action1 = ""
                action2 = ""


        # pierrot's turn
        if action1 in play:
            action2 = pierrot.update_models(play,copy.deepcopy(model_percepts2))
            print "-----------"+action2
            model_percepts1,model_percepts2,r1,r2 = world_update_play(action1,action2,previous)
            if action1==action2 and not action1 in previous:
                if len(previous)==len(play)-1:
                    del previous[0]
                previous.append(action1)
            cumrew1.append(r1)
            cumrew2.append(r2)
            action1 = ""
            action2 = ""
        else:
            action2 = pierrot.update_models(None,copy.deepcopy(model_percepts2))
            print "-----------"+action2
            if action1 and not (action2 in play):
                model_percepts1,model_percepts2 = world_update_communicate(action1,action2)
                cumrew1.append(0)
                cumrew2.append(0)
                action1 = ""
                action2 = ""

        if len(cumrew1)<j+1:
            cumrew1.append(0)
            cumrew2.append(0)
            

    CUMREW1+=(np.cumsum(np.array(cumrew1))+np.cumsum(np.array(cumrew2)))*0.5/float(N)
    if sum(cumrew1)>sum(cumrew2):
        CUMREW2+=np.cumsum(np.array(cumrew1))/float(N)
    else:
        CUMREW2+=np.cumsum(np.array(cumrew2))/float(N)

plt.plot(CUMREW1)
plt.plot(CUMREW2)
plt.plot(CUMREW1*2.-CUMREW2)
plt.show()
