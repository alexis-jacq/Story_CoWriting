#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model
import matplotlib.pyplot as plt

def create_bee(RT,RF,RPB,RP,RH):
    bee = model.Model()
    bee.add_cells(["trap","burn","hive","flower","pollen","honey","fullbee"])
    bee.add_actions(["left","right","take","give"])
    bee.set_rewards([["burn",1.,RT],["honey",1.,RH],["flower",1.,RF],["pollen",1.,RP],["fullbee",1.,RPB]])
    return bee

"""
trap = 1-abs(T-x) with T position of a trap (in -1:1)
hive = being inside the hive (boolean)
flower = 1-abs(F-x) with F position of a flower (in -1:1)
pollen = getting a pollen (boolean)
honey = giving a pollen to the hive (boolean)
fullbee = 1-abs(pb) with x position of bee carrying pollen (in 0:1)
the hive is at x=0

optimal behavior is to go to flower, to take pollen and to bring to the hive
if a flower is on position F with no trap between F and 0,
it brings a reward ([R(F)+R(PB)]*F*(F+1)/(2*XMAX) + R(P) + R(H))/(F*(F+1)+1)
"""

# PARAMETERS
XMAX = 10. # space radius
TMAX = 500 # time horizon
RT = -1.
RF = 0.2
RPB = 0.2
RH = 1.
RP = 0.1


# INIT
x = 0
pollen = False
T = -5.
F = 1.
cum_reward = []


def world_update(action):
    global x
    global pollen

    percepts = []
    reward = 0
    if action=="left":
        x-=1.

    if action=="right":
        x+=1.

    if action=="take":
        if x==F:
            percepts.append(("pollen",1))
            pollen=True
            reward += RP

    if action=="give":
        if x==0 and pollen:
            percepts.append(("pollen",-1))
            percepts.append(("honey",1))
            pollen=False
            reward += RH

    if x==T:
        percepts.append(("burn",1))
        reward += RT

    #percepts.append(("trap",1-abs(x-T)/XMAX))

    if pollen:
        percepts.append(("fullbee",1-abs(x)/XMAX))
        reward += RPB*(1-abs(x)/XMAX)*((1-abs(x)/XMAX)>0)
    else:
        percepts.append(("flower",1-abs(x-F)/XMAX))
        reward += RF*(1-abs(x-F)/XMAX)*((1-abs(x-F)/XMAX)>0)

    return percepts,reward


action = "right"
for j in range(1):
    bee = create_bee(RT,RF,RPB,RP,RH)
    for i in range(TMAX):
        p,r = world_update(action)
        action = bee.update(percepts=p)
        cum_reward.append(r)

print bee.n
print bee.matter

plt.plot(np.cumsum(np.array(cum_reward)))
plt.show()

