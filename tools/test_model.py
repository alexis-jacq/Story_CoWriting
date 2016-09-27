#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model2 as model
from mutualModelling import agent2 as agent
import matplotlib.pyplot as plt

def creat_robot():
    robot = model.Model("test")
    #robot.add_events(["fire","water","honey","d_energy","a1","a2","a3","n1","n2","n3"])
    robot.add_events(["fire","no_fire","honey","n1","n2","n3"])
    robot.add_actions(["a1","a2","a3"])
    robot.set_rewards([["fire",1.,-1.],["honey",1.,0.7]])
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
    print "----------------"

def print_rl():
    print "action:"
    print robot.action
    print "Q:"
    print robot.Q
    print "rewards:"
    print robot.rewards
    print "n:"
    print robot.n
    print "matter:"
    print robot.matter
    print '-----------------'


def world_response(action,setofevent):

    percepts = [(noise(),random.random())]
    #percepts = []
    new_set = set()

    energy = 0

    if "fire" in setofevent:
        if action != "a2":
            percepts.append(("fire",1))
            new_set.add("fire")
            energy -=1

    if action=="a1":
        percepts.append(("fire",1))
        new_set.add("fire")
        energy -=1

    if action=="a2":
        percepts.append(("no_fire",1))

    if action=="a3":
        if not ("fire" in setofevent):
            percepts.append(("honey",1))
            energy += 1

    return percepts,new_set,energy


m = 1000

action = "a1"
s = set()
cum_rew = []
optimal = []
ave_rew = np.zeros([m])
ave_reg = np.zeros([m])

n = 30
for j in range(n):
    robot = creat_robot()

    for i in range(m):
        p,s,e = world_response(action,s)
        #if i>m/2.:
        #    p = None
        action = robot.update(percepts=p)

        if random.random()>0.8:
            s.add("fire")
            optimal.append(-1)
        else:
            optimal.append(1)

        cum_rew.append(e)

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
