#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model

robot = model.Model()
robot.add_cells(["fire","burning","water","honey","d_energy","a1","a2","a3","n1","n2","n3"])

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
    print "times :"
    print robot.times
    print "weights :"
    print robot.weights
    print "activateds :"
    print robot.activateds
    print "----------------"

for i in range(1000):
    robot.update(percepts=[("a1",1),(noise(),1)])
    robot.update(percepts=[(noise(),1),("fire",1)])
    robot.update(reflexes=[("burning",1)], percepts=[(noise(),1)])
    robot.update(percepts=[("d_energy",-1),(noise(),1)])
print_robot()


