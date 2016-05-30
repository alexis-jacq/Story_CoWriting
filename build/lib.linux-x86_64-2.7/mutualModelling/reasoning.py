#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model
import matplotlib.pyplot as plt

class Agent:
    """agent able of first and 2nd mutual modelling reasoning"""
    def __init__(self,name,agents,percepts,actions):
        # name : string, name of the agent
        # agents : list of strings, itself + all other agents
        # percepts : list of strings
        # action : list of string

        self.name = name
        if name not in agents:
            agents.append(name)

        self.M = {} # Mutual Models constructed by the agent
        self.Id = {} # identification between 1st order and 2nd order point
                     # of view of a same agent : a.Id[b][x] = b viewed by x for a
                     # a.Id[b][c] = c,b
                     # a.M[c,b] := the model build by a of (the model of b by c)

        for agent1 in agents:
            self.M[agent1] = model.Model()
            self.M[agent1].add_cells([percepts])
            self.M[agent1].add_actions([actions])
            for agent2 in agents:
                self.M[agent1+','+agent2] = model.Model()
                self.M[agent1+','+agent2].add_cells([percepts])
                self.M[agent1+','+agent2].add_actions([actions])
                self.Id.setdefault(agent2,{})
                self.Id[agent2].setdefault(agent1,agent1+','+agent2)


