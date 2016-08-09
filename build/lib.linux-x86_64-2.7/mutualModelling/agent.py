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
    def __init__(self,name,agents,percepts,actions,rewards,instincts=None):
        # name : string, name of the agent
        # agents : list of strings, itself + all other agents
        # percepts : list of [strings,float]
        # action : list of string
        # rewards : list of [string,float,float] (observation,intensity,reward)
        # instincts : list of [string, float, string] (observation, intensity, action)

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
                self.M[agent].set_instincts(instincts)
            # 2nd order ToM:
            if agent!=self.name:
                name = self.name+"["+agent+';'+self.name+"]"
                self.M[agent+':'+self.name] = model.Model(name)
                self.M[agent+':'+self.name].add_events(percepts)
                self.M[agent+':'+self.name].add_actions(actions)
                # self.M[agent+':'+self.name].set_rewards(rewards)
                self.Id[agent+':'+self.name] = agent

    def update_models(self, restricted_actions=None, agents_obs=None, agents_actions=None):
        others_reward = 0
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
                        actions = agents_actions[agent]
                        # HACK: we should process all actions !!!
                        action = actions[0]


                if agent!=self.name:
                    r = self.M[agent].update_inverse(None, percepts=obs, last_action=action)
                    if agent in self.Id.values():
                        others_reward += r

        # estimation of errors by other agents about myself:
        self.others_error_reward, delta_reward = self.estimate_others_error_reward()
        self.others_error_knowledge, delta_knowledge = self.estimate_others_error_knowledge()

        #if delta_reward>0.1*np.random.rand():
        decision = None
        if False:
            # exagerate reinforcement learning behavior
            decision = self.M[self.name].update(restricted_actions,agents_obs[self.name],False,0)
        else:
            if agents_obs:
                if self.name in agents_obs:
                    decision = self.M[self.name].update(restricted_actions,agents_obs[self.name],True,others_reward)
            else:
                decision = self.M[self.name].update(restricted_actions,None,True,others_reward)

        return decision,agents_actions

        # TODO compute other's perception error of knowledge
        # TODO make prediction (update with no percepts) and compute prediction error


    def estimate_others_error_reward(self):
        # error by others about my reward:
        others_error_reward = 0
        for agent in self.Id:
            _,dist = model.diff_reward(self.M[self.name],self.M[agent])
            others_error_reward += dist
        delta_reward = abs(others_error_reward - self.others_error_reward)
        return others_error_reward, delta_reward


    def estimate_others_error_knowledge(self):
        # error by others about my knowledge:
        others_error_knowledge = 0
        for agent in self.Id:
            _,dist = model.diff_knowledge(self.M[self.name],self.M[agent])
            others_error_knowledge += dist
        delta_knowledge = abs(others_error_knowledge - self.others_error_knowledge)
        return others_error_knowledge, delta_knowledge
