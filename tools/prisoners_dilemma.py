#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model,agent
import matplotlib.pyplot as plt
import copy

def create_prisoner(name,all_names,RP,RW,RS):
    percepts = ["plonk","water","nothing","shock"]
    actions = ["cooperate","defect","ok","ko"]
    rewards = [["plonk",1.,RP],["water",1.,RW],["shock",1.,RS]]
    prisoner = agent.Agent(name,all_names,percepts,actions,rewards)
    return prisoner

"""
classic prisoners dilemma plus three options:
---------------------------------------------
before playing:
# say ok: your partner receive "ok" signal but does not know what it means for you
# say ko: your partner receive "ko" signal but does not know what it means for you
# say nothing
---------------------------------------------
as soon as a player choses to play (cooperate/defect),
the other has to play and can't communicate anymore
"""

# parameters
RP = 1.
RW = 0.7
RS = -1.

GM_values = np.array([[(0,0),(RP,RS)],[(RS,RP),(RW,RW)]])
GM_percepts = np.array([[("nothing","nothing"),("plonk","shock")],[("shock","plonk"),("water","water")]])

name1 = "robert"
name2 = "pierrot"
all_names = [name1,name2]

play = ["cooperate","defect"]

N = 1
n = 1000
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
    return model_percepts1,model_percepts2
    #return None,None

def world_update_play(action1,action2):
    p1 = []
    p2 = []
    b1 = int("cooperate" in action1)
    b2 = int("cooperate" in action2)
    r1 = GM_values[b1,b2,0]
    p1.append((GM_percepts[b1,b2,0],1))
    r2 = GM_values[b1,b2,1]
    p2.append((GM_percepts[b1,b2,1],1))
    # suppose no errors of perception:
    model_percepts1 = {name1:p1,name2:p2,name2+";"+name1:p1}
    model_percepts2 = {name2:p2,name1:p1,name1+";"+name2:p2}
    return model_percepts1,model_percepts2,r1,r2

for i in range(N):
    print i
    robert = create_prisoner(name1,all_names,RP,RW,RS)
    pierrot = create_prisoner(name2,all_names,RP,RW,RS)
    cumrew1 = []
    cumrew2 = []
    model_percepts1 = None
    model_percepts2 = None
    action1 = ""
    action2 = ""
    for j in range(n):
        #print j
        #print "model_percepts1 ="
        #print model_percepts1
        #print "model_percepts2 ="
        #print model_percepts2
        #print "============================"

        # robert's turn
        if action2 in play:
            action1 = robert.update_models(play,(model_percepts1))
            model_percepts1,model_percepts2,r1,r2 = world_update_play(action1,action2)
            cumrew1.append(r1)
            cumrew2.append(r2)
            action1 = ""
            action2 = ""
            #print "model_percepts2 modified ! !!!!!!"
            #print "================================="
        else:
            action1 = robert.update_models(None,copy.deepcopy(model_percepts1))
            if action1 and action2:
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
            model_percepts1,model_percepts2,r1,r2 = world_update_play(action1,action2)
            #print "model_percepts2 modified ! !!!!!!"
            #print "================================="
            cumrew1.append(r1)
            cumrew2.append(r2)
            action1 = ""
            action2 = ""
        else:
            action2 = pierrot.update_models(None,copy.deepcopy(model_percepts2))
            if action1 and action2:
                model_percepts1,model_percepts2 = world_update_communicate(action1,action2)
                #print "model_percepts2 modified ! !!!!!!"
                #print "=================================="
                cumrew1.append(0)
                cumrew2.append(0)
                action1 = ""
                action2 = ""

    CUMREW1+=np.cumsum(np.array(cumrew1))
    CUMREW2+=np.cumsum(np.array(cumrew2))

plt.plot(CUMREW1/float(N))
plt.plot(CUMREW2/float(N))
plt.show()
