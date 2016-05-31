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
                self.Id[agent+','+self.name] = agent


    def update(self,models_percepts):
        action = ""
        models_percepts.setdefault(self.name,[])

        concerned_models = set(models_percepts)

        while concerned_models:

            model = concerned_models.pop()

            if model!=self.name and model!="other" and model!="other,"+self.name:
                self.M[model].update(models_percepts[model])
                for percept in models_percepts[model]:
                    models_percepts[self.name].append((model+"_"+percept[0],percpt[1]))
                # no need to add self.name to concerned_models

                if model in self.Id:
                    name = "other,"+self.name
                    self.M[name].update(models_percepts[model])
                    for percept in models_percepts[model]:
                        models_percepts[self.name].append((name+"_"+percept[0],percept[1]))
                        models_percepts.setdefault(self.Id[model],[])
                        models_percepts[self.Id[model]].append((self.name+"_"+percept[0],percept[1]))
                        models_percepts["other"].append((self.name+"_"+percept[0],percept[1]))
                    concerned_models.add(self.Id[model])
                    concerned_models.add("other")

                else:
                    self.M["other"].update(models_percepts[model])
                    for percept in models_percepts[model]:
                        models_percepts[self.name].append(("other_"+percept[0],percpt[1]))
                    # no need to add self.name to concerned_models


            elif model=="other":
                self.M[model].update(models_percepts[model])
                for percept in models_percepts[model]:
                    models_percepts[self.name].append((model+"_"+percept[0],percpt[1]))

            elif model=="other,"+self.name:
                self.M[name].update(models_percepts[model])
                for percept in models_percepts[model]:
                    models_percepts[self.name].append((model+"_"+percept[0],percept[1]))
                    models_percepts["other"].append((self.name+"_"+percept[0],percept[1]))
                concerned_models.add("other")

        action=model.update(models_percepts[self.name])

        # TODO compute other's perception error
        # TODO make prediction (update with no percepts) and compute prediction error


