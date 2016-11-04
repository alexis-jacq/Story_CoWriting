#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model2 as model
from mutualModelling.model2 import diff_reward
#import matplotlib.pyplot as plt
import copy
import operator

class Agent:
    """agent able of 1st and 2nd mutual modelling reasoning"""
    def __init__(self,name,agents,percepts,actions,rewards,social_reward=0): # if inverse RL => rewards is just for self.name !!

        self.name = name
        if name not in agents:
            agents.append(name)

        self.M = {} # Mutual Models constructed by the agent
                    # a.M[b] := the model build by 'a' of 'b'
                    # a.M[c:b] := the model build by 'a' of ('b' seen by 'c')

        self.Id = {}# identification of the agent who is modelling me
                    # a.Id[b:a] = b 

        self.social_reward = social_reward

        self.prev_diff = 0
        self.social_curve = []

        for agent in set(agents):
            name = self.name+"["+agent+"]"
            self.M[agent] = model.Model(name)
            self.M[agent].add_events(percepts)
            self.M[agent].add_actions(actions)
            if agent==self.name:
                self.M[agent].set_rewards(rewards)
            if agent!=self.name:
                name = self.name+"["+agent+';'+self.name+"]"
                self.M[agent+':'+self.name] = model.Model(name)
                self.M[agent+':'+self.name].add_events(percepts)
                self.M[agent+':'+self.name].add_actions(actions)
                self.M[agent+':'+self.name].set_rewards(rewards)
                self.Id[agent+':'+self.name] = agent


    def update_models(self,possible_actions=None,models_percepts=None,model_actions=None):

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

                    if (model in self.Id.values()): # if it's (or not) about me
                        IR += self.social_reward * r
                        n+=1.

            diff = 0
            for agent in self.M:
                _,dist = diff_reward(self.M[self.name],self.M[agent])
                diff += dist
            d_error = np.sqrt((self.prev_diff - diff)**2)
            self.prev_diff = diff
            self.social_curve.append(diff)

            # 'c'est l'intention qui compte'
            decision = self.M[self.name].update(possible_actions,self_percepts,social_reward=IR/n, social_error=d_error)

            return decision
        else:
            return self.M[self.name].update(possible_actions,None,True)


    # display functions:
    #-------------------
    def show_learned_goals(self,agent):
        print self.M[agent].number_event
        print self.M[agent].number_action
        print self.M[agent].rewards

    def show_goals_error(self,agent):
        social_diffs,total_diffs = diff_reward(self.M[self.name],self.M[agent+":"+self.name])
        print social_diffs

    def plot_social_curve(self):
        return self.social_curve

    # social actions:
    #----------------
    def reward(self,agent):
        values = {}
        for action in self.M[self.name].action_number:
            if action in self.M[agent].event_number:
                obs_num = self.M[agent].event_number[action]
                values[action] = self.M[agent].rewards[obs_num]
        return max(values.iteritems(), key=operator.itemgetter(1))[0]

    def punish(self,agent):
        values = {}
        for action in self.M[self.name].action_number:
            if action in self.M[agent].event_number:
                obs_num = self.M[agent].event_number[action]
                values[action] = self.M[agent].rewards[obs_num]
        return min(values.iteritems(), key=operator.itemgetter(1))[0]