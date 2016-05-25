#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model
import matplotlib.pyplot as plt

def create_bee(RT,RF,RPB,RP,RH):
    bee = model.Model()
    bee.add_cells(["trap","burn","hive","flower","pollen","honey","fullbee","bee"])
    bee.add_actions(["left","right","take","give"])
    bee.set_rewards([["burn",1.,RT],["honey",1.,RH],["flower",1.,RF],["pollen",1.,RP],["fullbee",1.,RPB],["lose",1,-1.]])
    return bee

"""
trap = 1-abs(T-x) with T position of a trap (in -1:1)
hive = being inside the hive (boolean)
flower = 1-abs(F-x) with F position of a flower (in -1:1)
pollen = getting a pollen (boolean)
honey = giving a pollen to the hive (boolean)
fullbee = 1-abs(pb) with x position of bee carrying pollen (in 0:1)
bee = x pose of the bee
the hive is at x=0

optimal behavior is to go to flower, to take pollen and to bring to the hive
if a flower is on position F with no trap between F and 0,
it brings a reward ([R(F)+R(PB)]*F*(F+1)/(2*XMAX) + R(P) + R(H))/(F*(F+1)+1)
"""

# PARAMETERS
XMAX = 10. # space radius
TMAX = 3000 # time horizon
RT = -1.0
RF = 0.5
RPB = 0.5
RH = 1.
RP = 0.05


# INIT
x = 0
pollen = False
T = -1.
F = 3.
cum_reward = []
curve_x = []


def world_update(action):
    global x
    global pollen

    percepts = []
    reward = 0
    if action=="right" and x>-XMAX:

        if pollen and abs(x-1)<abs(x):

            percepts.append(("fullbee",1-abs(x-1)/XMAX))
            #reward += RPB*(1-abs(x-1)/XMAX)*((1-abs(x-1)/XMAX)>0)

        if (not pollen) and abs(x-1-F)<abs(x-F):

            percepts.append(("flower",1-abs(x-1-F)/XMAX))
            #reward += RF*(1-abs(x-1-F)/XMAX)*((1-abs(x-1-F)/XMAX)>0)
        x-=1.

    if action=="left" and x<XMAX:

        if pollen and abs(x+1)<abs(x):

            percepts.append(("fullbee",1-abs(x+1)/XMAX))
            #reward += RPB*(1-abs(x+1)/XMAX)*((1-abs(x+1)/XMAX)>0)

        if (not pollen) and abs(x+1-F)<abs(x-F):

            percepts.append(("flower",1-abs(x+1-F)/XMAX))
            #reward += RF*(1-abs(x+1-F)/XMAX)*((1-abs(x+1-F)/XMAX)>0)
        x+=1.

    if action=="take":
        if x==F:
            percepts.append(("pollen",1))
            pollen=True
            #reward += RP

    if action=="give":
        if x==0 and pollen:
            percepts.append(("pollen",-1))
            percepts.append(("honey",1))
            pollen=False
            reward += RH
        #if x!=0 and pollen:
            #percepts.append(("pollen",-1))
            #percepts.append(("lose",1))
            #pollen=False

    percepts.append((str(x),1))

    """if x==T:
        percepts.append(("burn",1))
        reward += RT"""

    print x
    print pollen
    curve_x.append(x)

    return percepts,reward


action = "right"
for j in range(1):
    bee = create_bee(RT,RF,RPB,RP,RH)
    for i in range(TMAX):
        p,r = world_update(action)
        if p:
            action = bee.update(percepts=p)
        else:
            action = bee.update()
        cum_reward.append(r)

print bee.n
print bee.matter

cumrew = np.cumsum(np.array(cum_reward))
regret = np.array(range(TMAX))/(2.*abs(F)+2)-cumrew

plt.plot(cumrew)
plt.plot(regret)
plt.show()
traj = np.array(curve_x)
plt.plot(traj[TMAX-500:-1])
plt.show()

