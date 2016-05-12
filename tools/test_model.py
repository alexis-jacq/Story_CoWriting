#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model
import matplotlib.pyplot as plt

def creat_robot():
    robot = model.Model()
    robot.add_cells(["fire","burning","water","honey","d_energy","a1","a2","a3","n1","n2","n3"])
    robot.add_actions(["a1","a2","a3"])
    robot.set_goal([["d_energy",1.,1.]])
    return robot

""" learning actions:
    -----------------
    when robot does a1 it receive a fire, then burn and lose energy
    when robot does a2 it receive water and stop the burn
    when robot does a3 it receive honey and win energy
    n1,n2 and n3 is noise and appear randomly
    d_energy is the derivative of energy
"""

def noise():
    r = random.uniform(0,1)
    if r>2/3.:
        return "n1"
    elif r>1/3.:
        return "n2"
    else:
        return "n3"

def print_hebbian():
    print "counts :"
    print robot.counts
    print "weights :"
    print robot.weights
    print "----------------"

def print_rl():
    print robot.Q
    print robot.V
    print robot.R
    print robot.n


def word_response(action,setofevent):
    global a1,a2,a3

    percepts = [(noise(),1)]
    reflexes = []
    new_set = set()

    energy = 0

    if action != "":
        percepts.append((action,1))
    if action=="a1":
        reflexes.append(("fire",1))
        new_set.add("burning")
    if action=="a2":
        reflexes.append(("water",1))
        new_set.add("water")
    if action=="a3":
        reflexes.append(("honey",1))
        new_set.add("honey")

    if "fire" in setofevent and not "water" in setofevent:
        reflexes.append(("burning",1))
        new_set.add("burning")
    if ("burning" in setofevent) and not ("water" in setofevent):
        if "honey" in setofevent:
            pass
        else:
            reflexes.append(("d_energy",-1))
            energy-=1
        new_set.add("burning")
    if "water" in setofevent:
        reflexes.append(("burning",-1))
    if "honey" in setofevent and not ("burning" in setofevent):
        reflexes.append(("d_energy",1))
        energy+=1

    return percepts,reflexes,new_set,energy

#print_rl()

m = 300

action = "a1"
s = set()
cum_rew = []
optimal = []
ave_rew = np.zeros([m])
ave_reg = np.zeros([m])

n = 1

for j in range(n):
    robot = creat_robot()
    for i in range(m):
        p,r,s,e = word_response(action,s)
        action = robot.update(percepts=p,reflexes=r)

        if random.random()>0.9:
            s.add("fire")
            optimal.append(-1)
        else:
            optimal.append(1)
        cum_rew.append(e)

        #print_rl()

    ave_rew += np.cumsum(np.array(cum_rew))
    ave_reg += np.cumsum(np.array(optimal)-np.array(cum_rew))
    cum_rew = []
    optimal = []

    if j%10==0:
        print j

#print_hebbian()
print_rl()

plt.plot(ave_rew/n)
plt.plot(ave_reg/n)
plt.show()

