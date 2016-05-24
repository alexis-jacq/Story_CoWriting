#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model
import matplotlib.pyplot as plt

def create_bee():
    bee = model.Model()
    bee.add_cells(["trap","hive","flower","pollen","honey","x"])
    bee.add_actions(["left","right","take","give"])
    bee.set_rewards([["trap",1.,-1.],["honey",1.,1.],["flower",1.,0.2],["pollen",1.,0.1]])
    return bee

"""
trap = x position of a trap related (in -1:1)
hive = being inside the hive (boolean)
flower = x position of a flower (in -1:1)
pollen = getting a pollen (boolean)
honey = giving a pollen to the hive (boolean)
x = x position of the bee (in -1:1)
the hive is at x=0
"""

def world_update():

