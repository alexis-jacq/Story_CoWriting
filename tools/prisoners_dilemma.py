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
    actions = ["cooperate","defect","1","2"]#,"3","4","5"]#,"6","7","8","9"]
    rewards = [["plonk",1.,RP],["water",1.,RW],["shock",1.,RS],['nothing',1,RN]]
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
RW = 1.1
RS = -1.
RN = 0

GM_values = np.array([[(RN,RN),(RP,RS)],[(RS,RP),(RW,RW)]])
GM_percepts = np.array([[("nothing","nothing"),("plonk","shock")],[("shock","plonk"),("water","water")]])

name1 = "robert"
name2 = "pierrot"
all_agents = [name1,name2]

play = ["cooperate","defect"]
say = ["1","2"]#,"3","4","5"]#,"6","7","8","9"]

N = 1
n = 1000

CUMREW1 = np.zeros(n)
CUMREW2 = np.zeros(n)


def world_update(action1,action2,play):
    p1 = []
    p2 = []
    r1 = 0
    r2 = 0
    if play:
        b1 = int("cooperate" in action1)
        b2 = int("cooperate" in action2)
        r1 = GM_values[b1,b2,0]
        p1.append((GM_percepts[b1,b2,0],1))
        r2 = GM_values[b1,b2,1]
        p2.append((GM_percepts[b1,b2,1],1))
    else:
        p1.append((action2,1))
        p2.append((action1,1))
    # suppose no errors of perception:   
    
    model_percepts1 = {name1:p1,name2:p2,name2+":"+name1:p1}
    model_percepts2 = {name2:p2,name1:p1,name1+":"+name2:p2}
    model_actions1 = {name2:action2,name2+":"+name1:action1}
    model_actions2 = {name1:action1,name1+":"+name2:action2}

    return model_percepts1,model_percepts2,model_actions1,model_actions2,r1,r2

for i in range(N):
    if i%10==0:
        print i
    robert = create_prisoner(name1,all_agents,RP,RW,RS)
    pierrot = create_prisoner(name2,all_agents,RP,RW,RS)
    cumrew1 = []
    cumrew2 = []
    model_percepts1 = None
    model_percepts2 = None
    model_actions1 = None
    model_actions2 = None
    action1 = ""
    action2 = ""
    for j in range(n):

        # say:
        
        action1 = robert.update_models(say,model_percepts1,model_actions1)
        action2 = pierrot.update_models(say,model_percepts2,model_actions2)
        print action1
        print action2
        print '-----'
        model_percepts1,model_percepts2,model_actions1,model_actions2,r1,r2 = world_update(action1,action2,False)
        
        # play:
        action1 = robert.update_models(play,model_percepts1,model_actions1)
        action2 = pierrot.update_models(play,model_percepts2,model_actions2)
        print action1
        print action2
        print '-----'
        model_percepts1,model_percepts2,model_actions1,model_actions2,r1,r2 = world_update(action1,action2,True)
        
        cumrew1.append(float(action1=='cooperate'))
        cumrew2.append(float(action2=='cooperate'))
        #cumrew1.append(r1)
        #cumrew2.append(r2)


    CUMREW1+=np.cumsum(np.array(cumrew1))/float(N)
    CUMREW2+=np.cumsum(np.array(cumrew2))/float(N)

pierrot.show_learned_goals('robert')
print pierrot.M['pierrot'].n
print'========'
robert.show_learned_goals('pierrot')
print robert.M['robert'].n
print '======='


plt.subplot(2, 1, 1)
#plt.plot(CUMREW1)
#plt.plot(CUMREW2)
plt.plot(0.5*(CUMREW1+CUMREW2)) # score of cooperation

plt.subplot(2, 1, 2)
plt.plot(0.5*(np.array(pierrot.plot_social_curve())+np.array(robert.plot_social_curve())))

plt.show()
