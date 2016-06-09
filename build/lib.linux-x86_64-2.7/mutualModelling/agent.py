#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model
from mutualModelling.model import diff_reward
import matplotlib.pyplot as plt
import copy

def softsign(x,n):
    y = abs(x)
    if y>1.:
        y=1.
    z = (1-y)**n
    return z*np.sign(x)

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

        self.prev_diff = 0

        if len(set(agents))>2 and not "other" in agents: # 'other' represents another agent in general"
            agents.append("other")

        for agent in set(agents):
            name = self.name+"["+agent+"]"
            self.M[agent] = model.Model(name)
            self.M[agent].add_cells(percepts)
            self.M[agent].add_actions(actions)
            if agent==self.name:
                self.M[agent].set_rewards(rewards)
            if agent!=self.name:
                name = self.name+"["+agent+';'+self.name+"]"
                self.M[agent+';'+self.name] = model.Model(name)
                self.M[agent+';'+self.name].add_cells(percepts)
                self.M[agent+';'+self.name].add_actions(actions)
                #self.M[agent+';'+self.name].set_rewards(rewards)
                self.Id[agent+';'+self.name] = agent


    def update_models(self,possible_actions=None,models_percepts=None,model_actions=None):
        #print possible_actions
        #print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
        #print self.name
        if models_percepts:
            models_percepts.setdefault(self.name,[])

            concerned_models = set(models_percepts)
            u = copy.deepcopy(models_percepts[self.name])

            #print "----------------------"
            #print self.name

            for model in concerned_models:
                action = None
                if model_actions:
                    if model in model_actions:
                        action = model_actions[model]
                test = False
                if self.name=="learner" and  model=="teacher":
                    print "======================================================="
                    print self.name+" think about "+model
                    test = True
                
                if model!=self.name:
                    self.M[model].update_inverse(possible_actions,percepts=models_percepts[model],last_action=action,testprint=test)



            #   while concerned_models:

            #       action = None
            #       model = concerned_models.pop()
            #       if model_actions:
            #           if model in model_actions:
            #               action = model_actions[model]
            #               #print model+" "+action
            #       test = False
            #       if self.name=="learner" and model=="teacher":
            #           print "======================================================="
            #           print self.name+" think about "+model
            #           test = True
            #       #print models_percepts
            #       #print concerned_models

            #       
            #       if model!=self.name:
            #           p = copy.deepcopy(models_percepts[model])
            #           self.M[model].update_inverse(possible_actions,percepts=models_percepts[model],last_action=action,testprint=test)
            #           models_percepts.pop(model)
            #           """
            #           for percept in p:
            #               u.append((model+"_"+percept[0],percept[1]))
            #           if model in self.Id:
            #               for percept in p:
            #                   models_percepts.setdefault(self.Id[model],[])
            #                   models_percepts[self.Id[model]].append((self.name+"_"+percept[0],percept[1]))

            #               concerned_models.add(self.Id[model])"""

            understood=0
            explore = True
            diff = 0
            for agent in self.Id:
                _,dist = diff_reward(self.M[self.name],self.M[agent])
                diff += dist
            if diff>self.prev_diff:
                explore = False
                #pass
            self.prev_diff = diff

            return self.M[self.name].update(possible_actions,u,explore)
        else:
            return self.M[self.name].update(possible_actions,None,True)

        # TODO compute other's perception error
        # TODO make prediction (update with no percepts) and compute prediction error

    def show_learned_rewards(self,agent):
        print self.M[agent].cell_number.inv
        print self.M[agent].action_number.inv
        if agent==self.name:
            print self.M[agent].R
        else:
            print self.M[agent].rewards

    def show_social_error(self,agent):
        social_diffs,total_diffs = diff_reward(self.M[self.name],self.M[agent+";"+self.name])
        print social_diffs

    """
    def be_kind(self): # do the action the other prefer you do
        for agent in self.
        for action in self.M[self.name].action_number:
            if action in self.M[agent


    def be_bad(self,agent): # do the action the other

    def analyse(self,agent,action):
    """
