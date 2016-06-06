#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model
import matplotlib.pyplot as plt
import copy

class Agent:
    """agent able of first and 2nd mutual modelling reasoning"""
    def __init__(self,name,agents,percepts,actions,rewards): # if inverse RL => rewards is just for self.name !!!
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
            self.M[agent].add_cells(percepts)
            self.M[agent].add_actions(actions)
            if agent==self.name:
                self.M[agent].set_rewards(rewards)
            if agent!=self.name:
                self.M[agent+';'+self.name] = model.Model()
                self.M[agent+';'+self.name].add_cells(percepts)
                self.M[agent+';'+self.name].add_actions(actions)
                #self.M[agent+';'+self.name].set_rewards(rewards)
                self.Id[agent+';'+self.name] = agent


    def update_models(self,possible_actions=None,models_percepts=None,model_actions=None):
        #print possible_actions
        if models_percepts:
            models_percepts.setdefault(self.name,[])

            concerned_models = set(models_percepts)
            u = copy.deepcopy(models_percepts[self.name])

            #print "----------------------"
            #print self.name
            while concerned_models:

                action = None
                model = concerned_models.pop()
                if model_actions:
                    if model in model_actions:
                        action = model_actions[model]
                        #print model+" "+action
                #print model
                #print models_percepts
                #print concerned_models

                if model!=self.name:
                    p = copy.deepcopy(models_percepts[model])
                    self.M[model].update_inverse(possible_actions,percepts=models_percepts[model],last_action=action)
                    models_percepts.pop(model)
                    for percept in p:
                        u.append((model+"_"+percept[0],percept[1]))
                    if model in self.Id:
                        for percept in p:
                            models_percepts.setdefault(self.Id[model],[])
                            models_percepts[self.Id[model]].append((self.name+"_"+percept[0],percept[1]))

                        concerned_models.add(self.Id[model])

            return self.M[self.name].update(possible_actions,u)
        else:
            return self.M[self.name].update(possible_actions,None)

        # TODO use invese reinforcement learning for other agents'update for cing what was the previous other agent's action
        # TODO compute other's perception error
        # TODO make prediction (update with no percepts) and compute prediction error

    def show_learned_rewards(self,agent):
        print self.M[agent].cell_number.inv
        print self.M[agent].action_number.inv
        print self.M[agent].rewards

    def show_social_error(self,agent):
        print model.diff_reward(self.M[self.name],self.M[agent+";"+self.name])

