#!/usr/bin/env python
# coding: utf-8

"""
library of functions/algorithms to build, update and compaire models of different agents built by a robot.
"""

import numpy as np
import random
import operator
import copy

""" GLOBAL PARAMETERS """
# hebbian learning:
#==================

# reinforcement learning:
#========================
THETA1 = 10 # chose action (exponent for softmax pulling
THETA2 = 10 # chose perception
DISCOUNT = 0.8 # discount for the impact of futur on the temporal diff algo #0

""" functions for random pulling"""
#----------------------------------
def random_pull_dict(distribution): # dist. is a dictionnary key->value
     if distribution:
        proba = np.array(distribution.values())*1.
        proba = proba/np.sum(proba)
        return np.random.choice(distribution.keys(),1,p=proba)[0]
     else:
         return None

def softmax(distribution): # dist. is a list (np.array) of values
    global THETA1
    if list(distribution):
        if THETA1<=0:
            THETA1 = 1e-15
        proba = np.exp(THETA1*np.array(distribution))
        proba = proba/np.sum(proba)
        return np.random.choice(len(distribution),1,p=proba)[0]
    else:
        return None


""" object Model """
#-------------------
class Model:
    """ an object 'Model' representing hebbian-inspired network that encode dynamics between events representing concepts learned by an agent."""
    def __init__(self,name, network=None, activateds=None, modifieds=None):

        # name:
        self.name = name

        # events encoding events:
        #-----------------------
        self.nb_events = 0
        self.intensities = np.zeros([self.nb_events]) # list of event's intensity between 0 and 1 (intensity or truth)
        self.event_number = {} # each event is numeroted {event_id <--> event_number}
        self.number_event = {} # each event is numeroted {event_id <--> event_number}
        self.last_event = ""
        self.last_intensity = 0

        # hebbian learning (world's causality):
        #--------------------------------------
        self.counts = np.zeros([0,0,0]) # (action, event1, event2) --> correlation : count the close activations for hebbian learning

        # reinforcement learning (action):
        #---------------------------------
        self.action = None
        self.expected = 0
        self.rewards = np.zeros([0]) # reward associated with goals (0 if no objective) 
        self.action_number = {} # set of events encoding actions
        self.number_action = {} # set of events encoding actions
        self.nb_actions = 0
        # for TD learning in event space:
        self.Q = np.zeros([0,0]) # (event, action) --> value : indirect reward value learned by association ~ like QLearning with TD
        self.n = np.zeros([0,0]) # (event, action) --> number : occurence of couple (event-action)
        self.matter = np.ones([0]) # importance of events (based on V)

        # IRL and understandable behavior:
        #---------------------------------
        self.be_explicite = True
        self.expected_action = np.zeros([0])
        self.expected_state = np.zeros([0])
        self.expected_intensity = np.zeros([0]) # 
        self.explicite_rewards = np.zeros([0]) # that another could learn given my behaviour


    """ functions for creating/updating/using models """
    #--------------------------------------------------------------
    def add_events(self, events_id):
        if isinstance(events_id, list) or isinstance(events_id, tuple):
            number = self.nb_events
            for event_id in events_id:
                
                if event_id not in self.event_number:
                    self.event_number[event_id] = number
                    self.number_event[number] = event_id
                    number += 1

                    new_intensities = np.zeros([number])
                    new_intensities[:self.nb_events] = self.intensities
                    self.intensities = new_intensities

                    new_counts = np.zeros([self.nb_actions,number, number])
                    new_counts[:,:self.nb_events,:self.nb_events] = self.counts
                    self.counts = new_counts

                    new_matter = np.ones([number])
                    new_matter[:self.nb_events] = self.matter
                    self.matter = new_matter

                    new_EA = -np.ones([number])
                    new_EA[:self.nb_events] = self.expected_action
                    self.expected_action = new_EA

                    new_ES = np.zeros([number])
                    new_ES[:self.nb_events] = self.expected_state
                    self.expected_state = new_ES

                    new_EI = np.zeros([number])
                    new_EI[:self.nb_events] = self.expected_intensity
                    self.expected_intensity = new_EI

                    new_rewards = np.zeros([number])
                    new_rewards[:self.nb_events] = self.rewards
                    self.rewards = new_rewards

                    new_erewards = np.zeros([number])
                    new_erewards[:self.nb_events] = self.explicite_rewards
                    self.explicite_rewards = new_erewards

                    new_Q = np.zeros([number, self.nb_actions])
                    new_Q[:self.nb_events,:self.nb_actions] = self.Q
                    self.Q = new_Q

                    new_n = np.zeros([number, self.nb_actions])
                    new_n[:self.nb_events,:self.nb_actions] = self.n
                    self.n = new_n

                    self.nb_events = number

    def add_actions(self, events_id):
        if isinstance(events_id, list) or isinstance(events_id, tuple):
            self.add_events(events_id)
            number = self.nb_actions
            for event_id in events_id:
                if event_id not in self.action_number:
                    self.action_number[event_id] = number
                    self.number_action[number] = event_id
                    number += 1

                    new_counts = np.zeros([number, self.nb_events, self.nb_events])
                    new_counts[:self.nb_actions,:,:] = self.counts
                    self.counts = new_counts

                    new_Q = np.zeros([self.nb_events, number])
                    new_Q[:,:self.nb_actions] = self.Q
                    self.Q = new_Q

                    new_n = np.zeros([self.nb_events, number])
                    new_n[:,:self.nb_actions] = self.n
                    self.n = new_n

                    self.nb_actions = number

    def set_rewards(self, goals):
        for goal in goals:
            event_id = goal[0]
            value = goal[1]
            reward = goal[2]
            if event_id not in self.event_number:
                self.add_events([event_id])
            if value>1:
                value=1.
            if value<-1:
                value=-1.
            self.rewards[self.event_number[event_id]] = reward

    def set_instincts(self, obs_actions): # ~ a-priori knowledge
        for obs_action in obs_actions:
            event_id = obs_action[0]
            value = obs_action[1]
            action = obs_action[2]
            if event_id not in self.event_number:
                self.add_events([event_id])
            if action not in self.action_number:
                self.add_actions([action])
            if value>1:
                value=1.
            if value<-1:
                value=-1.
            event_num = self.event_number[event_id]
            action_num = self.action_number[action]
            self.Q[event_num,action_num] = 1.
            # if EMA of TD:
            self.V[event_num,action_num] = 1.


    def perceive_new_event(self, percepts, total_reward, elligibles):

        for percept in percepts:
            if not (percept in self.event_number):
                self.add_events([percept[0]])

            percept_id = percept[0]
            percept_val = percept[1]
            percept_num = self.event_number[percept_id]

            self.intensities[self.event_number[percept_id]] = percept_val

            elligibles.setdefault(percept_id,0)
            elligibles[percept_id] = np.exp(THETA2*self.matter[self.event_number[percept_id]])

            if self.action and self.last_event :
                total_reward += self.rewards[percept_num]*percept_val

                father = self.last_event
                son = percept_id
                intensity_father = self.intensities[self.event_number[self.last_event]]
                intensity_son = percept_val
                action = self.action
                self.hebbian_learning(father,son,action,intensity_father,intensity_son)

        return total_reward, elligibles


    def update(self, possible_actions=None, percepts=None, social_reward=0, social_error=0 ):
        elligibles = {}
        total_reward = social_reward

        # REASONING (not yet):
        #===========
        #if self.last_event and not percepts:
        #    elligibles, new_intensities = self.think_new_event(elligibles, new_intensities)

        # PERCEPTION:
        #============
        # could add an action "force_reasoning" where the robot doesnot do the perception loop
        # like someone closing eyes in order to reason
        if percepts:
            total_reward, elligibles = self.perceive_new_event(percepts, total_reward, elligibles)

        # UPDATES:
        #=========
        # stochastic election of incoming events:
        new_obs = random_pull_dict(elligibles)

        '''
        if total_reward>1:
            total_reward=1.
        if total_reward<-1:
            total_reward=-1.
        '''

        # reinf. learning:
        #=================
        if self.action and percepts:
            self.reinforcement_learning(new_obs,total_reward)

        # decision making:
        #=================
        self.last_event = new_obs
        decision = self.decision_making(possible_actions, social_error)
        
        return decision

    def update_inverse(self, possible_actions=None, percepts=None, last_action=None):
        '''imagine the update of other agents'''

        elligibles = {}
        total_reward = 0
        if last_action:
            if not (last_action in self.action_number):
                self.add_actions([last_action])
            if not (last_action in self.event_number):
                self.add_events([last_action])
            self.action = last_action

        # REASONING (not yet):
        #===========
        #if self.last_event and not percepts:
        #    elligibles, new_intensities = self.think_new_event(elligibles, new_intensities)

        # PERCEPTION:
        #============
        if percepts:
            total_reward, elligibles = self.perceive_new_event(percepts, total_reward, elligibles)

        # UPDATES:
        #=========
        # stochastic election of incoming events:
        new_obs = random_pull_dict(elligibles)

        '''
        if total_reward>1:
            total_reward=1.
        if total_reward<-1:
            total_reward=-1.
        '''

        # inverse reinf. learning:
        #=========================
        if self.last_event:
            self.inverse_learning()

        # reinf. learning:
        #=================
        if self.action and percepts:
            self.reinforcement_learning(new_obs,total_reward)

        # DECISION:
        #==========
        #prediction = self.decision_making(possible_actions) # not used yet

        # last update:
        self.last_event = new_obs

        return total_reward


    def hebbian_learning(self, event1, event2, action, intensity1, intensity2):
        num_event1 = self.event_number[event1]
        num_event2 = self.event_number[event2]
        num_act = self.action_number[action]
        correlation = intensity1*intensity2

        # competitive correlations:
        s = np.sum(self.counts[num_act,num_event1,:])
        v = self.counts[num_act,num_event1,num_event2]
        self.counts[num_act,num_event1,:] *= s/(s+correlation+1e-5)
        self.counts[num_act,num_event1,num_event2] = (s*v+correlation)/(s+correlation+1e-5)


    def decision_making(self, possible_actions=None, social_error=0):
        last_state = 0
        last_intensity = 0
        if self.last_event:
            last_state = self.event_number[self.last_event]
            last_intensity = self.intensities[last_state]
        else:
            last_state = np.argmax(np.random.rand(self.nb_events))
            last_intensity = self.intensities[last_state]

        noise = np.random.rand(len(self.Q[last_state,:]))*1e-15
        values = self.Q[last_state,:]*last_intensity + noise
        new_values = -np.Infinity*np.ones(len(values))

        if possible_actions:
            indices = []
            for action in possible_actions:
                indices.append(self.action_number[action])
            new_values[np.array(indices)]=values[np.array(indices)]
        else:
            new_values = values


        #if social_error<0.1:
        #    self.be_explicite = False
        if np.sum(np.sum(self.n))<300 or np.sum(np.sum(self.n))>700:
        #if True:
        #if False:
            # normal behavior:
            choice = softmax(new_values)

        else:
            self.be_explicite = True
            # try to show just the extrinsic goals ?
            # understandable behavior:
            '''remember last time'''
            expected_action = int(self.expected_action[last_state])
            expected_state = int(self.expected_state[last_state])
            expected_intensity = int(self.expected_intensity[last_state])

            if self.rewards[expected_state]*expected_intensity>1.1*np.random.rand():
                '''repeat action'''
                choice = self.expected_action[last_state]
            else:
                '''change action'''
                new_values[int(self.expected_action[last_state])]=-np.Infinity
                #choice = np.argmax(new_values)
                choice = softmax(new_values)

        self.expected = self.Q[last_state,int(choice)]*last_intensity
        self.action = self.number_action[choice]
        return self.action


    def reinforcement_learning(self, new_event, reward):
        if self.last_event:
            # last state
            last_state = self.event_number[self.last_event]
            last_intensity = self.intensities[last_state]

            # action
            action = self.action_number[self.action]

            # new state:
            new_state = self.event_number[new_event]
            new_intensity = self.intensities[new_state]

            # classic Q:
            new_values = self.Q[new_state,:]*new_intensity
            reach = np.max(new_values)

            # TD learning:
            TD = (reward + DISCOUNT*reach - self.expected)
            n = self.n[last_state,action]+1.

            # classic Qlearning
            self.Q[last_state,action] = (n*self.Q[last_state,action] + TD)/(n+1.)

            # importance of the event
            self.matter[new_state] = (n*(self.matter[new_state]) + abs(TD))/(n+1.)

            # understandable behavior
            self.expected_action[last_state] = action
            self.expected_state[last_state] = new_state
            self.expected_intensity[last_state] = new_intensity
            self.explicite_rewards[last_state] = (n*(self.matter[new_state]) + reward)/(n+1.)

            self.n[last_state,action] += 1.



    def inverse_learning(self):
        if self.last_event and self.action:
            # action:
            action = self.action_number[self.action]
            last_state = self.event_number[self.last_event]

            n = self.n[last_state,action]
            s = np.sum(self.n[last_state,:])

            expected_state = int(self.expected_state[last_state])
            expected_intensity = self.expected_intensity[last_state]
            '''
            if action == self.expected_action[last_state]:
                self.rewards[expected_state] = 0.99*self.rewards[expected_state] + 0.01
                #pass
            else:
                self.rewards[expected_state] = 0.99*self.rewards[expected_state] - 0.01
            '''
            if self.be_explicite:
                if action == self.expected_action[last_state]:
                    self.rewards[expected_state] = (1-1/(np.sqrt(1+self.n[last_state,action])))*self.rewards[expected_state] + 1/(np.sqrt(1+self.n[last_state,action]))
                    #pass
                else:
                    self.rewards[expected_state] = (1-1/(np.sqrt(1+self.n[last_state,action])))*self.rewards[expected_state] - 1/(np.sqrt(1+self.n[last_state,action]))
            

            # should be proportionnal to errors/ surprise

# static functions (of multiple models):
#---------------------------------------

def diff_reward(model1, model2):
    tot_dist = 0
    event_diff = {}
    for event_id in set(model1.event_number).intersection(set(model2.event_number)):
        event_num1 = model1.event_number[event_id]
        event_num2 = model2.event_number[event_id]
        # this distance function is arbitrary, could be L2, L3 etc...
        dist = np.sum(np.abs(model1.rewards[event_num1]-model2.rewards[event_num2]))
        event_diff.setdefault(event_id,dist)
        tot_dist += dist
    return event_diff,tot_dist

def diff_Q(model1, model2):
    tot_dist = 0
    event_diff = {}
    for event_id in set(model1.event_number).intersection(set(model2.event_number)):
        event_num1 = model1.event_number[event_id]
        event_num2 = model2.event_number[event_id]
        # this distance function is arbitrary, could be L2, L3 etc...
        dist = np.sum(np.abs(model1.Q[event_num1,:]-model2.Q[event_num2,:]))
        event_diff.setdefault(event_id,dist)
        tot_dist += dist
    return event_diff,tot_dist

def diff_knowledge(model1,model2):
    tot_dist = 0
    event_diff = {}
    for event_id in set(model1.event_number).intersection(set(model2.event_number)):
        event_num1 = model1.event_number[event_id]
        I1 = model1.intensities[event_id]
        I2 = model2.intensities[event_id]
        # this distance function is arbitrary, could be L2, L3 etc...
        dist = np.sum(np.abs(I1-I2))*model1.matter[event_num1]
        event_diff.setdefault(event_id,dist)
        tot_dist += dist
    return event_diff,tot_dist
