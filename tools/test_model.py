#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model

robot = model.Model()
robot.add_cells(["fire","burning","water","honey","d_energy","a1","a2","a3","n1","n2","n3"])
robot.add_actions(["a1","a2","a3"])
robot.set_goal([["d_energy",1.]])

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

def print_robot():
    print "counts :"
    print robot.counts
    #print "times :"
    #print robot.times
    print "weights :"
    print robot.weights[0]
    #print "cell_number :"
    #print robot.cell_number
    print "activateds :"
    print robot.activateds
    print "intensities :"
    print robot.intensities
    print "----------------"

def word_response(action,setofevent):
    percepts = [(noise(),1)]
    reflexes = []
    new_set = set()
    if action != "":
        percepts.append((action,1))
    if action=="a1":
        percepts.append(("fire",1))
        new_set.add("burning")
    if action=="a2":
        percepts.append(("water",1))
        new_set.add("water")
    if action=="a3":
        percepts.append(("honey",1))
        new_set.add("honey")
    if "fire" in setofevent:
        reflexes.append(("burning",1))
        new_set.add("burning")
    if "burning" in setofevent:
        percepts.append(("d_energy",-1))
    if "water" in setofevent:
        percepts.append(("burning",-1))
    if "honey" in setofevent:
        percepts.append(("d_energy",1))
    return percepts,reflexes,new_set

action = "a1"
s = set()
for i in range(10):
    p,r,s = word_response(action,s)
    action = robot.update(percepts=p,reflexes=r)
print_robot()


