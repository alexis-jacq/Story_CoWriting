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

        # TODO : need to learn the dynamic between agent's models [like counts/cor]

        if len(set(agents))>2 and not "other" in agents: # 'other' represents another agent in general"
            agents.append("other")

        for agent in set(agents):
            self.M[agent] = model.Model()
            self.M[agent].add_cells([percepts])
            self.M[agent].add_actions([actions])
            if agent!=self.name:
                self.M[agent+','+self.name] = model.Model()
                self.M[agent+','+self.name].add_cells([percepts])
                self.M[agent+','+self.name].add_actions([actions])
                self.Id.setdefault(self.name,[])
                self.Id[self.name].append(agent+','+self.name)


    def update(self,models_percepts):
        action = ""
        models_percepts.setdefault(self.name,[])

        for model in models_percepts:

            if model!=self.name and model!="other" and model!="other,"+self.name:
                self.M[model].update(models_percepts[model])
                #models_percepts[self.name].append(...

                if model in self.Id[self.name]:
                    name = "other,"+self.name
                    self.M[name].update(models_percepts[model])
                #if model in self.Id["other"]:
                #    self.M["other,other"].update(models_percepts[model])
                if model in self.Id:
                    self.M["other"].update(models_percepts[model])

            elif model!=self.name:
                self.M["other"].update(models_percepts[model])

        action=model.update(models_percepts[self.name])

        # TODO compute other's perception error
        # TODO make prediction (update with no percepts) and compute prediction error


