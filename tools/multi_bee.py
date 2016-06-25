#!/usr/bin/env python
# coding: utf-8

import numpy as np
import copy
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.animation as manimation

from bidict import bidict
from mutualModelling import model,agent

SIZE = 100
MID = int(SIZE/2.)-1
VISUAL_MAP = np.zeros((SIZE,SIZE,3))
MAP = np.zeros((SIZE,SIZE))
BEE_MAP = np.zeros((SIZE,SIZE))
NB_BEE = 1
LEN_VEW = 3

objects_num = bidict({"hive_1":1,"hive_2":2,"bee_1":3,"bee_2":4,"flower_1":5,"flower_2":6})

all_names = []
for i in range(NB_BEE):
    all_name.append("bee_1_"+str(i))
    all_name.append("bee_2_"+str(i))

def init_MAP():
    global MAP
    z = np.random.rand(SIZE,SIZE)
    MAP[z>0.1] =  objects_num["flower_1"]
    MAP[z>0.2] =  objects_num["flower_2"]
    MAP[MID,0] = objects_num["hive_1"]
    MAP[MID,SIZE-1] = objects_num["hive_2"]

def create_bee_1(name,all_names):
    percepts = ["hive_1","hive_2","bee_1","bee_2","flower_1","flower_2"]
    actions = ["up","down","left","right"]
    rewards = [["food",1.,1.],["hurt",1.,-1.]]
    bee_1 = agent.Agent(name,all_names,percepts,actions,rewards)
    return bee_1

def create_bee_2(name,all_names):
    percepts = ["hive_1","hive_2","bee_1","bee_2","flower_1","flower_2"]
    actions = ["go","left","right"]
    rewards = [["food",1.,1.],["hurt",1.,-1.]]
    bee_2 = agent.Agent(name,all_names,percepts,actions,rewards)
    return bee_2

class Bee:
    def __init__(self, name, kind, x, y, d):
        self.name = name
        self.kind = kind
        if kind==1:
            self.agent = create_bee_1(name,all_names)
        if kind==2:
            self.agent = create_bee_2(name,all_names)
        self.x = x
        self.y = y
        self.d = d

    def percept(self):
        obs = []
        for i in range(LEN_VEW):
            for j in range(LEN_VEW):
                if self.d == 0:
                    ii = self.x+i
                    jj = self.y+j
                if self.d ==1:
                    ii = self.x-i
                    jj = self.y+j
                if self.d ==2:
                    ii = self.x+i
                    jj = self.y-j
                if self.d ==3:
                    ii = self.x-i
                    jj = self.y-j
                obs.append((objects_num.inv[MAP[ii%SIZE,jj%SIZE]],1.)
                obs.append((objects_num.inv[BEE_MAP[ii%SIZE,jj%SIZE]],1.)
        if objects_num.inv[MAP[x,y]] == "flower_"+str(self.kind):
            obs.append(("food",1.))



BEES = []
for name in all_names:
    if name[4]=='1':
        BEES.append(Bee(name,1,MID,0,np.random.choice([0,1,2,3])))
    if name[4]=='2':
        BEES.append(Bee(name,2,MID,0,np.random.choice([0,1,2,3])))
