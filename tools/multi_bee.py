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
OBS_MAP = np.zeros((SIZE,SIZE,3))
MAP = np.zeros((SIZE,SIZE))
BEE_MAP = np.zeros((SIZE,SIZE))
NB_BEE = 10
LEN_VEW = 10
BEES = []

objects_num = bidict({"nothing":0,"hive_1":1,"hive_2":2,"bee_1":3,"bee_2":4,"flower_1":5,"flower_2":6})

all_names = []


def init_MAP():
    global MAP
    z = np.random.rand(SIZE,SIZE)
    MAP[z<0.2] =  objects_num["flower_1"]
    MAP[z<0.1] =  objects_num["flower_2"]
    MAP[MID,0] = objects_num["hive_1"]
    MAP[MID,SIZE-1] = objects_num["hive_2"]

def create_bee_1(name,all_names):
    percepts = ["hive_1","hive_2","bee_1","bee_2","flower_1","flower_2"]
    actions = ["go","left","right"]
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
        self.dx = d[0]
        self.dy = d[1]

    def update(self, bees_map):
        global MAP
        global OBS_MAP
        obs = {self.name:[]}
        """
        mid_len = int(LEN_VEW/2.)
        for i in range(LEN_VEW):
            for j in range(LEN_VEW):
                ii = self.x + self.dx*i + (1-abs(self.dx))*(i-mid_len)
                jj = self.y + self.dy*j + (1-abs(self.dy))*(j-mid_len)
                if bees_map[ii%SIZE,jj%SIZE]>0:
                    di = self.dx*i + (1-abs(self.dx))*(i-mid_len)
                    dj = self.dy*j + (1-abs(self.dy))*(j-mid_len)
                    intensity = 1./(1. + di**2+dj**2)
                    obs[self.name].append((objects_num.inv[bees_map[ii%SIZE,jj%SIZE]],1.))
        """
        for i in np.arange(LEN_VEW)+1:
            ii = self.x + self.dx*i
            jj = self.y + self.dy*i
            di = self.dx*i
            dj = self.dy*i
            intensity = np.sqrt(1./(di**2+dj**2+0.))
            OBS_MAP[ii%SIZE,jj%SIZE] = [intensity,intensity,intensity]
            if bees_map[ii%SIZE,jj%SIZE]>0:
                obs[self.name].append((objects_num.inv[bees_map[ii%SIZE,jj%SIZE]],intensity))


        if objects_num.inv[MAP[self.x,self.y]] == "flower_"+str(self.kind):
            obs[self.name].append(("food",1.))
            MAP[self.x,self.y] = 0

        if objects_num.inv[MAP[self.x,self.y]] == "flower_"+str(self.kind%2 + 1):
            obs[self.name].append(("hurt",1.))
            MAP[self.x,self.y] = 0

        action = self.agent.update_models(None,obs,None,"MM1")

        dx = self.dx
        dy = self.dy
        if action=="left":
            self.dx = -dy
            self.dy = dx
        if action=="right":
            self.dx = dy
            self.dy = -dx
        if action=="go":
            self.x = (self.x+self.dx)%SIZE
            self.y = (self.y+self.dy)%SIZE


def update_bees():
    global BEES
    for bee in BEES:
        #print bee.name+" "+str(bee.x)+" "+str(bee.y)+" "+str(bee.dx)+" "+str(bee.dy)
        #print bee.agent.M[bee.name].Q[:,:,1]
        #print "--------------------"
        bee.update(vew_bees())

def vew_bees():
    bees_map = copy.deepcopy(MAP)
    for bee in BEES:
        bees_map[bee.x,bee.y] = objects_num["bee_"+str(bee.kind)]
    return bees_map

def add_flowers():
    global MAP
    nb_flowers = np.sum(MAP==objects_num["flower_1"]) + np.sum(MAP==objects_num["flower_1"])
    if nb_flowers<SIZE:
        x = int(np.random.rand()*SIZE)%SIZE
        y = int(np.random.rand()*SIZE)%SIZE
        if np.random.rand()>0.5:
            MAP[x,y] = objects_num["flower_1"]
        else:
            MAP[x,y] = objects_num["flower_2"]

    MAP[MID,0] = objects_num["hive_1"]
    MAP[MID,SIZE-1] = objects_num["hive_2"]

def vew_map():
    global VISUAL_MAP
    global OBS_MAP
    bees_map = vew_bees()
    VISUAL_MAP[bees_map==0] = [0.,0.,0.]
    VISUAL_MAP[bees_map==1] = [0.,1.,0.]
    VISUAL_MAP[bees_map==2] = [0.,0.,1.]
    VISUAL_MAP[bees_map==3] = [0.3,1.,0.3]
    VISUAL_MAP[bees_map==4] = [0.3,0.3,1.]
    VISUAL_MAP[bees_map==5] = [1.,1.,0.]
    VISUAL_MAP[bees_map==6] = [1.,0.,1.]
    VISUAL_MAP += OBS_MAP
    VISUAL_MAP[VISUAL_MAP>1] = 1.
    OBS_MAP = np.zeros((SIZE,SIZE,3))


fig = plt.figure()
im = plt.imshow(VISUAL_MAP,interpolation='nearest')

def update(*args):
    global BEES
    global MAP
    global VISUAL_MAP

    update_bees()
    add_flowers()
    vew_map()

    im = plt.imshow(VISUAL_MAP,interpolation='nearest')
    return im,



if __name__=='__main__':

    init_MAP()

    for i in range(NB_BEE):
        all_names.append("bee_1_"+str(i))
        all_names.append("bee_2_"+str(i))

    d = {1:(1,0),2:(0,1),3:(-1,0),4:(0,-1)}

    for name in all_names:
        if name[4]=='1':
            BEES.append(Bee(name,1,MID,0,d[np.random.choice([1,2,3,4])]))
        if name[4]=='2':
            BEES.append(Bee(name,2,MID,SIZE-1,d[np.random.choice([1,2,3,4])]))

    ani = manimation.FuncAnimation(fig, update, interval=50, blit=True)
    plt.show()
