#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model
from mutualModelling.model import diff_reward
import matplotlib.pyplot as plt
import copy
import operator

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
        self.social_curve = []

        #if len(set(agents))>2 and not "other" in agents: # 'other' represents another agent in general"
        #    agents.append("other")

        for agent in set(agents):
            name = self.name+"["+agent+"]"
            self.M[agent] = model.Model(name)
            self.M[agent].add_cells(percepts)
            self.M[agent].add_actions(actions)
            if agent==self.name:
                self.M[agent].set_rewards(rewards)
                # intrinsic reward for compassion
                # self.M[agent].set_rewards([["others_rew",1.,1.],["others_rew",-1.,-1.]])
            if agent!=self.name:
                name = self.name+"["+agent+';'+self.name+"]"
                self.M[agent+';'+self.name] = model.Model(name)
                self.M[agent+';'+self.name].add_cells(percepts)
                self.M[agent+';'+self.name].add_actions(actions)
                #self.M[agent+';'+self.name].set_rewards(rewards)
                self.Id[agent+';'+self.name] = agent


    def update_models(self,possible_actions=None,models_percepts=None,model_actions=None,case="MM1"):

        IR = 0
        n=0.1
        if models_percepts:
            models_percepts.setdefault(self.name,[])

            concerned_models = set(models_percepts)
            self_percepts = copy.deepcopy(models_percepts[self.name])

            for model in concerned_models:
                action = None
                if model_actions:
                    if model in model_actions:
                        action = model_actions[model]

                if model!=self.name:
                    r = self.M[model].update_inverse(possible_actions,percepts=models_percepts[model],last_action=action)
                    if model in self.Id.values():
                    #if self.name=="learner":
                    #if True:
                        IR+=0*r
                        n+=1.

            understood=0
            explore = True
            diff = 0
            for agent in self.Id:
                _,dist = diff_reward(self.M[self.name],self.M[agent])
                diff += dist

            if case=="MM2":
                if abs(diff-self.prev_diff)>0.1*np.random.rand():
                    explore = False
            if case=="MM1_understand":
                explore=False
            if case=="MM1":
                explore=True

            self.prev_diff = diff
            self.social_curve.append(diff)

            self_r = self.self_reward(self_percepts)

            #decision = self.M[self.name].update(possible_actions,self_percepts,explore,intrinsic=IR/n)

            
            if self_r>0 and explore:
                gift = []
                for agent in self.Id.values():
                    gift.append(self.reward(agent))
                decision = self.M[self.name].update(gift,self_percepts,explore,intrinsic=IR/n)

            #if self_r<0:
            #    gift = []
            #    for agent in self.Id.values():
            #        gift.append(self.punish(agent))
            #    decision = self.M[self.name].update(gift,self_percepts,explore,intrinsic=OR/n)

            if self_r<=0 or not explore:
                decision = self.M[self.name].update(possible_actions,self_percepts,explore,intrinsic=IR/n)
            

            return decision
        else:
            return self.M[self.name].update(possible_actions,None,True)

        # TODO compute other's perception error
        # TODO make prediction (update with no percepts) and compute prediction error


    # display functions:
    #-------------------
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

    def plot_social_curve(self):
        return self.social_curve

    # social actions:
    #----------------
    def reward(self,agent):
        values = {}
        for action in self.M[self.name].action_number:
            if action in self.M[agent].cell_number:
                obs_num = self.M[agent].cell_number[action]
                values[action] = self.M[agent].rewards[obs_num,1]
        #print self.name+" rewards "+agent+" with "+max(values.iteritems(), key=operator.itemgetter(1))[0]
        return max(values.iteritems(), key=operator.itemgetter(1))[0]

    def punish(self,agent):
        values = {}
        for action in self.M[self.name].action_number:
            if action in self.M[agent].cell_number:
                obs_num = self.M[agent].cell_number[action]
                values[action] = self.M[agent].rewards[obs_num,1]
        #print self.name+" rewards "+agent+" with "+max(values.iteritems(), key=operator.itemgetter(1))[0]
        return min(values.iteritems(), key=operator.itemgetter(1))[0]

    # estimate my reward:
    #--------------------
    def self_reward(self,percepts):
        r = 0
        for percept in percepts:
            if percept[0] in self.M[self.name].cell_number:
                obs_num = self.M[self.name].cell_number[percept[0]]
                intensity = percept[1]
                r += self.M[self.name].R[obs_num,intensity]
        return r
