#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model,agent
import matplotlib.pyplot as plt
import copy

def create_prisoner(name,all_names,RP,RW,RS):
    percepts = ["plonk","water","nothing","shock"]
    actions = ["cooperate","defect","cooperate_&_te","cooperate_&_both","defect_&_spy","defect_&_send_message","defect_&_both"]
    rewards = [["plonk",1.,RP],["water",1.,RW],["shock",1.,RS]]
    prisoner = agent.Agent(name,all_names,percepts,actions,rewards)
    return prisoner

"""
classic prisoners dilemma plus three options:
---------------------------------------------
# spy: you can know what was the previous action of your partner
# send_message: you can tell your futur action to your partner
# both: spy and send message
---------------------------------------------
"""

# parameters
RP = 1.
RW = 0.5
RS = -1.

GM_values = np.array([[(0,0),(RP,RS)],[(RS,RP),(RW,RW)]])
GM_percepts = np.array([[("nothing","nothing"),("plonk","shock")],[("shock","plonk"),("water","water")]])

name1 = "robert"
name2 = "pierrot"
all_names = [name1,name2]

N = 1
n = 1000

def world_update(action1,action2):
    p1 = []
    p2 = []
    b1 = "cooperate" in action1
    b2 = "cooperate" in action2
    r1 = GM_values[b1,b2,0]
    p1.append(GM_percepts[b1,b2,0])
    r2 = GM[b1,b2,1]
    p2.append(GM_percepts[b1,b2,1])
    

for i in range(N):
    robert = create_prisoner(name1,all_names,RP,RW,RS)
    pierrot = create_prisoner(name2,all_names,RP,RW,RS)
    for j in range(n):
        

