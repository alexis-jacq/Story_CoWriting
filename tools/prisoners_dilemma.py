#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model2 as model
from mutualModelling import agent2 as agent
import matplotlib.pyplot as plt
import copy

def create_prisoner(name,all_names,RP,RW,RS):
    percepts = ["plonk","water","nothing","shock"]
    actions = ["cooperate","defect","ok","ko"]
    rewards = [["plonk",1.,RP],["water",1.,RW],["shock",1.,RS]]
    prisoner = agent.Agent2(name,all_names,percepts,actions,rewards)
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
RW = 0.1
RS = -1.

GM_values = np.array([[(0,0),(RS,RP)],[(RP,RS),(RW,RW)]])
GM_percepts = np.array([[("nothing","nothing"),("shock","plonk")],[("plonk","shock"),("water","water")]])

name1 = "robert"
name2 = "pierrot"
all_agents = [name1,name2]

play = ["cooperate","defect"]

N = 1
n = 500
CUMREW1 = np.zeros(n)
CUMREW2 = np.zeros(n)


def world_update(action1,action2):
    p1 = []
    p2 = []
    b1 = int("cooperate" in action1)
    b2 = int("cooperate" in action2)
    r1 = GM_values[b1,b2,0]
    p1.append((GM_percepts[b1,b2,0],1))
    r2 = GM_values[b1,b2,1]
    p2.append((GM_percepts[b1,b2,1],1))
    # suppose no errors of perception:
    model_percepts1 = {name1:p1,name2:p2,name2+":"+name1:p1}
    model_percepts2 = {name2:p2,name1:p1,name1+":"+name2:p2}
    model_actions1 = {name2:action2,name2+":"+name1:action1}
    model_actions2 = {name1:action1,name1+":"+name2:action2}
    return model_percepts1,model_percepts2,model_actions1,model_actions2,r1,r2

for i in range(N):
    if i%10==0:
        print i
    robert = create_prisoner(name1,all_names,RP,RW,RS)
    pierrot = create_prisoner(name2,all_names,RP,RW,RS)
    cumrew1 = []
    cumrew2 = []
    model_percepts1 = None
    model_percepts2 = None
    model_actions1 = None
    model_actions2 = None
    action1 = ""
    action2 = ""
    for j in range(n):

        action1 = robert.update_models(None,model_percepts1,model_actions1)
        action2 = pierrot.update_models(None,model_percepts2,model_actions2)
        model_percepts1,model_percepts2,model_actions1,model_actions2,r1,r2 = world_update(action1,action2)
        cumrew1.append(r1)
        cumrew2.append(r2)


    CUMREW1+=np.cumsum(np.array(cumrew1))/float(N)
    CUMREW2+=np.cumsum(np.array(cumrew2))/float(N)

plt.plot(CUMREW1)
plt.plot(CUMREW2)
plt.plot(CUMREW1-CUMREW2)
plt.show()
