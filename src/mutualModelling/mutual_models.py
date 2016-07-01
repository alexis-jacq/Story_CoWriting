#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model
from mutualModelling.model import diff_reward
import matplotlib.pyplot as plt
import copy
import operator

class MMagent:
    """agent able of first and 2nd mutual modelling reasoning"""
    def __init__(self,name,agents,percepts,actions,rewards):
        # name : string, name of the agent
        # agents : list of strings, itself + all other agents
        # percepts : list of strings
        # action : list of string

        self.name = name
        if name not in agents:
            agents.append(name)

        self.M = {} # Mutual Models constructed by the agent
                    # a.M[b] := the model build by 'a' of agent 'b'
                    # a.M[c:b] := the model build by 'a' of (agent 'b' seen by agent 'c')
        self.Id = {} # identification between 1st order and 2nd order point
                     # of view of a same agent : a.Id[b:x] = b

        # misunderstanding:
        # of myself by others:
        self.others_error_reward = 0
        self.others_error_knowledge = 0
        # of others by me:
        self.MM1_prediction_error = 0
        self.MM2_prediction_error = 0

        for agent in set(agents):
            # 1st order ToM:
            name = self.name+"["+agent+"]"
            self.M[agent] = model.Model(name)
            self.M[agent].add_events(percepts)
            self.M[agent].add_actions(actions)
            if agent==self.name:
                self.M[agent].set_rewards(rewards)
            # 2nd order ToM:
            if agent!=self.name:
                name = self.name+"["+agent+';'+self.name+"]"
                self.M[agent+':'+self.name] = model.Model(name)
                self.M[agent+':'+self.name].add_events(percepts)
                self.M[agent+':'+self.name].add_actions(actions)
                # self.M[agent+':'+self.name].set_rewards(rewards)
                self.Id[agent+':'+self.name] = agent

    def update_models(self, restricted_actions=None, agents_obs=None, agents_actions=None):
        if agents_obs or agents_actions:
            concerned_agents = set(agents_obs).union(set(agents_actions))
            for agent in concerned_agents:
                obs = None
                if agents_obs:
                    if agent in agents_obs:
                        obs = agents_obs[agent]
                action = None
                if agents_actions:
                    if agent in agents_actions:
                        action = agents_actions[agent]

                if agent!=self.name:
                    r = self.M[agent].update_inverse(None, percepts=agents_obs[agent], last_action=action)

        others_error_reward = 0
        for agent in self.Id:
            _,dist = diff_reward(self.M[self.name],self.M[agent])
            others_error_reward += dist
        delta_reward = others_error_reward - self.others_error_reward
        self.others_error_reward = others_error_reward

        if delta>0:
            decision = self.M[self.name].update(restricted_actions,self_percepts,False,0)
        else:
            decision = self.M[self.name].update(restricted_actions,self_percepts,True,0)

        return decision

        # TODO compute other's perception error of knowledge
        # TODO make prediction (update with no percepts) and compute prediction error
