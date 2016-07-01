#!/usr/bin/env python
# coding: utf-8

"""
library of functions/algorithms to build, update and compaire models of different agents built by a robot.
"""

import numpy as np
import random
import operator
from bidict import bidict
import copy

""" GLOBAL PARAMETERS """
# hebbian learning:
#==================
FIRE_TIME = 10 # time a event is activated
ETA1 = 0.9 # for EMA of the correlation between intensity of signals
# if delayed hebbian: GAMMA = 0.1 # time discount for learning

# reinforcement learning:
#========================
THETA1 = 10 # chose action (exponent for softmax pulling
THETA2 = 20 # chose perception
ETA2 = 0.8
DISCOUNT = 0.99 # discount for the impact of futur on the temporal diff algo

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
    if list(distribution):
        proba = np.exp(THETA1*distribution)
        proba = proba/np.sum(proba)
        return np.random.choice(len(distribution),1,p=proba)[0]
    else:
        return None


""" object Model """
#-------------------
class Model:
    """ an object 'Model' representing hebbian-inspired network that encode dynamics between events representing concepts learned by an agent."""
    def __init__(self,name, network=None, activateds=None, modifieds=None):

        global FIRE_TIME

        # name:
        self.name = name

        # events encoding events:
        #-----------------------
        self.intensities = {} # list of event's intensity between -1 and 1 (intensity or truth)
        self.nb_events = 0
        self.activateds = [] # list of activated events, the first is the most recently activated (contains by default the event encoding the empty concept)
        self.old_intensities = []
        self.modifieds = set() # for each input from exterior (percept) event intensities are modified once
                               # it makes the differrence between the flow of reasoning and real perception
        self.event_number = bidict() # each event is numeroted {event_id <--> event_number}

        # hebbian learning (world's causality):
        #--------------------------------------
        self.counts = np.zeros([0,0,0,2,2]) # count the close activations for hebbian learning

        # reinforcement learning (action):
        #---------------------------------
        self.action = None
        self.expected = 0
        self.rewards = np.zeros([0,2]) # reward associated with goals (0 if no objective)
        self.action_number = bidict() # set of events encoding actions
        self.nb_actions = 0
        # for TD learning with fuzzy states:
        self.Q = np.zeros([0,0,2]) # indirect reward value learned by association ~ like QLearning with TD
        self.V = np.zeros([0,0,2]) # for actor-critic method
        self.n = np.zeros([0,0,2])
        self.matter = np.ones([0,2]) # importance of events (based on V)
        self.R = np.zeros([0,2]) # estimation of direct reward

        # IRL and understandable behavior:
        #---------------------------------
        self.EA = -np.ones([0,2])
        self.ES = np.zeros([0,2])
        self.EI = np.zeros([0,2])


    """ functions for creating/updating/using models """
    #--------------------------------------------------------------
    def add_activated(self, event):
        if self.activateds:
            if len(self.activateds)==FIRE_TIME:
                for i in range(len(self.activateds)-1):
                    self.activateds[i] = self.activateds[i+1]
                self.activateds[-1] = event
            else:
                self.activateds.append(event)
        else:
            self.activateds.append(event)

    def add_intensity(self, intensity):
        if self.old_intensities:
            if len(self.old_intensities)==FIRE_TIME:
                for i in range(len(self.old_intensities)-1):
                    self.old_intensities[i] = self.old_intensities[i+1]
                self.old_intensities[-1] = intensity
            else:
                self.old_intensities.append(intensity)
        else:
            self.old_intensities.append(intensity)

    def add_perceived(self, val):
        if self.perceiveds:
            if len(self.perceiveds)==FIRE_TIME:
                for i in range(len(self.perceiveds)-1):
                    self.perceiveds[i] = self.perceiveds[i+1]
                self.perceiveds[-1] = val
            else:
                self.perceiveds.append(val)
        else:
            self.perceiveds.append(val)

    def add_events(self, events_id):
        if isinstance(events_id, list) or isinstance(events_id, tuple):
            number = self.nb_events
            for event_id in events_id:
                self.intensities.setdefault(event_id,0)
                if event_id not in self.event_number:
                    self.event_number[event_id] = number
                    number += 1

                    new_counts = np.zeros([self.nb_actions,number, number,2,2])
                    new_counts[:,:self.nb_events,:self.nb_events,:,:] = self.counts
                    self.counts = new_counts
            #self.Q[last_state,action,int(last_intensity>0)] += 0.1*TD

                    new_matter = np.ones([number,2])
                    new_matter[:self.nb_events,:] = self.matter
                    self.matter = new_matter

                    new_R = np.zeros([number,2])
                    new_R[:self.nb_events,:] = self.R
                    self.R = new_R

                    new_EA = -np.ones([number,2])
                    new_EA[:self.nb_events,:] = self.EA
                    self.EA = new_EA

                    new_ES = np.zeros([number,2])
                    new_ES[:self.nb_events,:] = self.ES
                    self.ES = new_ES

                    new_EI = np.zeros([number,2])
                    new_EI[:self.nb_events,:] = self.EI
                    self.EI = new_EI

                    new_rewards = np.zeros([number,2])
                    new_rewards[:self.nb_events,:] = self.rewards
                    self.rewards = new_rewards

                    new_Q = np.zeros([number, self.nb_actions,2])
                    new_Q[:self.nb_events,:self.nb_actions,:] = self.Q
                    self.Q = new_Q

                    new_V = np.zeros([number, self.nb_actions,2])
                    new_V[:self.nb_events,:self.nb_actions,:] = self.V
                    self.V = new_V

                    new_n = np.zeros([number, self.nb_actions,2])
                    new_n[:self.nb_events,:self.nb_actions,:] = self.n
                    self.n = new_n

                    self.nb_events = number

    def add_actions(self, events_id):
        if isinstance(events_id, list) or isinstance(events_id, tuple):
            self.add_events(events_id)
            number = self.nb_actions
            for event_id in events_id:
                if event_id not in self.action_number:
                    self.action_number[event_id] = number
                    number += 1

                    new_counts = np.zeros([number, self.nb_events, self.nb_events,2,2])
                    new_counts[:self.nb_actions,:,:,:,:] = self.counts
                    self.counts = new_counts

                    new_Q = np.zeros([self.nb_events, number,2])
                    new_Q[:,:self.nb_actions,:] = self.Q
                    self.Q = new_Q

                    new_V = np.zeros([self.nb_events, number,2])
                    new_V[:,:self.nb_actions,:] = self.V
                    self.V = new_V

                    new_n = np.zeros([self.nb_events, number,2])
                    new_n[:,:self.nb_actions,:] = self.n
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
            self.rewards[self.event_number[event_id],int(value>0)] = reward

    def think_new_event(self, elligibles, new_intensities):

        last_intensity = self.old_intensities[-1]
        last_event = self.activateds[-1]
        num_last_event = self.event_number[last_event]
        num_action = self.action_number[self.action]

        # max over positive/negative new_intensities:
        noise = np.random.rand(len(self.counts[0,0,:,0,0]))/1000.

        """
        # rational :
        max_pos = np.max(self.counts[num_action,num_last_event,:,int(last_intensity>0),1] + noise)
        max_neg = np.max(self.counts[num_action,num_last_event,:,int(last_intensity>0),0] + noise)

        if max_pos>=max_neg:
            new_event_num = np.argmax(self.counts[num_action,num_last_event,:,int(last_intensity>0),1] + noise)
            new_intensity = np.max(self.counts[num_action,num_last_event,:,int(last_intensity>0),1] + noise)
        else:
            new_event_num = np.argmax(self.counts[num_action,num_last_event,:,int(last_intensity>0),0] + noise)
            new_intensity = -np.max(self.counts[num_action,num_last_event,:,int(last_intensity>0),0] + noise)
        """

        # hopefull :
        dream_act_pos = np.max(self.counts[:,:,np.argmax(self.R[:,1]),1,1],0)
        dream_act_neg = np.max(self.counts[:,:,np.argmax(self.R[:,1]),0,1],0)

        if max(dream_act_pos)>=max(dream_act_neg):
            new_event_num = np.argmax(dream_act_pos + noise)
            new_intensity = 1.
        else:
            new_event_num = np.argmax(dream_act_neg + noise)
            new_intensity = -1

        new_event = self.event_number.inv[new_event_num]
        elligibles.setdefault(new_event,0)
        elligibles[new_event] = np.exp(THETA2*self.matter[new_event_num,int(new_intensity>0)]*abs(new_intensity))
        new_intensities.setdefault(new_event,0)
        new_intensities[new_event] = new_intensity

        return elligibles, new_intensities


    def perceive_new_event(self,percepts,total_reward,elligibles):

        for percept in percepts:
            if not (percept in self.event_number):
                self.add_events([percept[0]])

            percept_id = percept[0]
            percept_val = percept[1]
            percept_num = self.event_number[percept_id]

            self.intensities[percept_id] = percept_val

            elligibles.setdefault(percept_id,0)
            elligibles[percept_id] = np.exp(THETA2*self.matter[self.event_number[percept_id],int(percept_val>0)])

            if self.action and self.old_intensities:
                total_reward += self.rewards[percept_num,int(percept_val>0)]*np.abs(self.old_intensities[-1])

                father = self.activateds[-1]
                son = percept_id
                intensity_father = self.old_intensities[-1]
                intensity_son = percept_val
                action = self.action
                self.hebbian_learning(father,son,action,intensity_father,intensity_son)

        return total_reward, elligibles


    def update(self, possible_actions=None, percepts=None, explore=True, intrinsic=0):

        # FIND THE NEXT ACTIVATED:
        elligibles = {}
        new_intensities = {}

        # REASONING:
        #===========
        if self.old_intensities and not percepts:
            elligibles, new_intensities = self.think_new_event(elligibles, new_intensities)


        # PERCEPTION:
        #============
        # could add an action "force_reasoning" where the robot doesnot do the perception loop
        # like someone closing eyes in order to reason
        tot_reward = intrinsic
        if percepts:
            tot_reward, elligibles = self.perceive_new_event(percepts, tot_reward, elligibles)


        # UPDATES:
        #=========
        # stochastic election of incoming active event:
        #print percepts
        next_activated = random_pull_dict(elligibles)
        """if percepts :
            print "obs : "+str(next_activated)
        else:
            print "think : "+str(next_activated)"""

        if tot_reward>1:
            tot_reward=1.
        if tot_reward<-1:
            tot_reward=-1.

        # new intensities:
        for event in new_intensities:
            if event not in self.modifieds:
                self.intensities[event] = new_intensities[event]
                self.modifieds.add(event)

        # TODO: loop on the previous percept in past to make reinforcement with delay

        # action learning:
        if self.action and percepts:
            self.reinforcement_learning(next_activated,tot_reward)

        # new activated event
        if next_activated:
            self.add_activated(next_activated)
            self.add_intensity(self.intensities[next_activated])

        # DECISION:
        #==========
        if possible_actions:
            return self.decision_making(possible_actions,explore)
        else:
            return self.decision_making(None,explore)


    def hebbian_learning(self, event1, event2, action, I1, I2):
        num_event1 = self.event_number[event1]
        num_event2 = self.event_number[event2]
        num_act = self.action_number[action]

        s = np.sum(self.counts[num_act,num_event1,:,int(I1>0),:])
        v = self.counts[num_act,num_event1,num_event2,int(I1>0),int(I2>0)]
        self.counts[num_act,num_event1,:,int(I1>0),:] *= s/(s+1.)
        self.counts[num_act,num_event1,num_event2,int(I1>0),int(I2>0)] = (s*v+1.)/(s+1.)


    def decision_making(self, possible_actions=None, explore=True):
        state = 0
        I = 0
        if self.activateds:
            state = self.event_number[self.activateds[-1]]
            I = self.old_intensities[-1]

        # TODO exploration based on convergence/difficulty to reach a state

        values = self.Q[state,:,int(I>0)]*np.abs(I)+np.random.rand(len(self.Q[state,:,int(I>0)]))/1000.
        new_values = -np.Infinity*np.ones(len(values))

        if possible_actions:
            indices = []
            for action in possible_actions:
                indices.append(self.action_number[action])
            new_values[np.array(indices)]=values[np.array(indices)]
        else:
            new_values = values

        if explore or self.EA[state,int(I>0)]==-1:
            choice = softmax(new_values)
        else:
            # understandable behavior:
            expected_state = int(self.ES[state,int(I>0)])
            expected_intensity = int(self.EI[state,int(I>0)]>0)

            if self.R[expected_state,expected_intensity]>np.random.rand():
                choice = self.EA[state,int(I>0)]
            else:
                new_values[int(self.EA[state,int(I>0)])]=-np.Infinity
                #choice = np.argmax(new_values)
                choice = softmax(new_values)

        # EMA:
        #self.expected = np.max(self.V[state,:,int(I>0)]*np.abs(I))
        self.expected = self.V[state,int(choice),int(I>0)]*np.abs(I)
        # Q:
        # self.expected = self.Q[state,choice,int(I>0)]*np.abs(I)
        self.action = self.action_number.inv[choice]
        return self.action


    def reinforcement_learning(self,new_activated,reward):
        if self.activateds and self.action and new_activated:

            # last state:
            action = self.action_number[self.action]
            last_state = self.event_number[self.activateds[-1]]
            last_intensity = self.old_intensities[-1]

            # new state:
            new_state = self.event_number[new_activated]
            new_intensity = self.intensities[new_activated]
            """
            # classic Q:
            new_values = self.Q[new_state,:,int(new_intensity>0)]*np.abs(new_intensity)
            """
            # expect EMA of TD:
            new_values = self.V[new_state,:,int(new_intensity>0)]*np.abs(new_intensity)

            reach = np.max(new_values)

            # TD learning:
            TD =  reward + DISCOUNT*reach - self.expected
            n = self.n[last_state,action,int(last_intensity>0)]+1.

            # classic Qlearning
            self.Q[last_state,action,int(last_intensity>0)] = (n*self.Q[last_state,action,int(last_intensity>0)] + TD)/(n+1.)

            self.n[last_state,action,int(last_intensity>0)] += 1.
            self.matter[new_state,int(new_intensity>0)] = (n*(self.matter[new_state,int(new_intensity>0)]) + abs(TD))/(n+1.)
            self.R[new_state,int(new_intensity>0)] = (n*(self.R[new_state,int(new_intensity>0)]) + reward)/(n+1.)

            # EMA of TD
            self.V[last_state,action,int(last_intensity>0)] = ETA2*self.V[last_state,action,int(last_intensity>0)] + (1-ETA2)*TD

            # understandable behavior
            self.EA[last_state,int(last_intensity>0)] = action
            self.ES[last_state,int(last_intensity>0)] = new_state
            self.EI[last_state,int(last_intensity>0)] = int(new_intensity>0)
            """
            print "last "+str(self.activateds[-1])+" "+str(last_intensity)
            print "act "+ str(self.action)
            print "new "+str(new_activated)+" "+str(new_intensity)
            print "rew "+str(TD)
            print "======================"
            """


    def update_inverse(self, possible_actions=None, percepts=None, last_action=None):

        # FIND THE NEXT ACTIVATED:
        elligibles = {}
        new_intensities = {}

        if last_action:
            if not last_action in self.action_number:
                self.add_actions([last_action])
            self.action = last_action

        # REASONING:
        #===========
        if self.old_intensities and not percepts:
            elligibles, new_intensities = self.think_new_event(elligibles, new_intensities)

        # PERCEPTION:
        #============
        # could add an action "force_reasoning" where the robot doesnot do the perception loop
        # like someone closing eyes in order to reason
        tot_reward = 0
        if percepts:
            tot_reward, elligibles = self.perceive_new_event(percepts, tot_reward, elligibles)

        # UPDATES:
        #=========
        # stochastic election of incoming active event:
        next_activated = random_pull_dict(elligibles)

        if len(self.activateds)>0:
            last_activated = self.activateds[-1]
            last_intensity = self.old_intensities[-1]
            self.inverse_learning(last_activated,last_intensity)

        if self.action:
            self.reinforcement_learning(next_activated,tot_reward)

        # new intensities:
        for event in new_intensities:
            if event not in self.modifieds:
                self.intensities[event] = new_intensities[event]
                self.modifieds.add(event)

        # new activated event
        if next_activated:
            self.add_activated(next_activated)
            self.add_intensity(self.intensities[next_activated])

        # DECISION:
        #==========
        if possible_actions:
            return tot_reward#self.decision(possible_actions)
        else:
            return tot_reward#self.decision()



    def inverse_learning(self,last_activated,last_intensity):
        if self.activateds and self.action:
            # action:
            action = self.action_number[self.action]
            last_state = self.event_number[last_activated]

            n = self.n[last_state,action,int(last_intensity>0)]
            s = np.sum(self.n[last_state,:,int(last_intensity>0)])

            expected_state = int(self.ES[last_state,int(last_intensity>0)])
            expected_intensity = self.EI[last_state,int(last_intensity>0)]

            if action == self.EA[last_state,int(last_intensity>0)]:
                self.rewards[expected_state,int(expected_intensity>0)] = 0.9*self.rewards[expected_state,int(expected_intensity>0)] + 0.1
            elif self.EA[last_state,int(last_intensity>0)]>=0:
                self.rewards[expected_state,int(expected_intensity>0)] = 0.9*self.rewards[expected_state,int(expected_intensity>0)] - 0.1


# static functions (of multiple models):
#---------------------------------------

def diff_reward(model1, model2):
    tot_dist = 0
    event_diff = {}
    for event_id in model1.event_number:# & model2.intensities:
        if event_id in model2.event_number:
            event_num1 = model1.event_number[event_id]
            event_num2 = model2.event_number[event_id]
            # this distance function is arbitrary, could be L2, L3 etc...
            dist = np.sum(np.abs(model1.R[event_num1,:]-model2.rewards[event_num2,:]))#* np.abs(model1.R[event_num1,:]))#*matter
            event_diff.setdefault(event_id,dist)
            tot_dist += dist
    return event_diff,tot_dist

def diff_knowledge(model1,model2):
    return 0
