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
STIFFNESS = 3 # how I expect the most likely event
FIRE_TIME = 10 # time a cell is activated
COUNT_MAX = 0. # plasticity
THRESHOLD = 2. # start to learn when something is recurrent
MIN = 0.00001 # minimal transition probability
GAMMA = 0.1 # time discount for learning

# reinforcement learning:
THETA1 = 30#5 # chose action (exponent for softmax pulling
THETA2 = 20#8 chose perception
DISCOUNT = 0.7 # discount for the impact of futur on the temporal diff algo

""" functions for random pulling"""
#----------------------------------
def stochastic_compare_stiffness(c1, c2):
    global STIFFNESS
    a,b = c1
    c,d = c2
    s = np.abs(b)**STIFFNESS + np.abs(d)**STIFFNESS
    r = random.uniform(0,s)
    return 1 if r>np.abs(b)**STIFFNESS else -1

def stochastic_compare_couples(c1, c2):
    global STIFFNESS
    a,b = c1
    c,d = c2
    s = np.abs(b) + np.abs(d)
    r = random.uniform(0,s)
    return 1 if r>np.abs(b) else -1

def stochastic_compare(x, y):
    s = np.abs(x) + np.abs(y)
    r = random.uniform(0,s)
    return 1 if r>np.abs(x) else -1

def random_pull_list(distribution): # dist. is a list (np.array) of values
    if list(distribution):
        couples = zip(range(len(distribution)),distribution)
        sorted_couples = sorted(couples,cmp=stochastic_compare_stiffness)
        return sorted_couples[0][0], sorted_couples[0][1]
    else:
        return None

def random_pull_dict(distribution): # dist. is a dictionnary key->value
     if distribution:
         sorted_dist = sorted(distribution.items(), key=operator.itemgetter(1), cmp=stochastic_compare)
         return sorted_dist[0][0]
     else:
         return None

def softmax(distribution): # dist. is a list (np.array) of values
    if list(distribution):
        expo = np.exp(distribution*THETA1)
        exponorm = expo/np.sum(expo)
        couples = zip(range(len(exponorm)),exponorm)
        sorted_couples = sorted(couples,cmp=stochastic_compare_couples)
        return sorted_couples[0][0], sorted_couples[0][1]
    else:
        return None


""" object Model """
#-------------------
class Model:
    """ an object 'Model' representing hebbian-inspired network that encode dynamics between cells representing concepts learned by an agent."""
    def __init__(self, network=None, activateds=None, modifieds=None):

        global FIRE_TIME

        # DEFAULT:

        # cells encoding events:
        #-----------------------
        self.intensities = {} # list of cell's intensity between -1 and 1 (intensity or truth)
        self.nb_cells = 0
        self.activateds = [] # list of activated cells, the first is the most recently activated (contains by default the cell encoding the empty concept)
        self.perceiveds = [] # are cells activated because perception (1) or because reasoning (0) ?
        self.old_intensities = []
        self.modifieds = set() # for each input from exterior (percept or percept) cell intensities are modified once
                               # it makes the differrence between the flow of thought and real perception
        self.cell_number = bidict() # each cell is numeroted {cell_id <--> cell_number}

        # hebbian learning (world's causality):
        #--------------------------------------
        self.counts = np.zeros([0,0,0]) # count the close activations for hebbian learning
        self.cor = np.zeros([0,0,0,2]) # corelation with positive intensity

        # reinforcement learning (action):
        #---------------------------------
        self.action = None
        self.expected = 0
        self.goals = np.zeros([0]) # goal value (1 or -1) for each cells
        self.rewards = np.zeros([0,2]) # reward associated with goals (0 if no objective)
        self.action_number = bidict() # set of cells encoding actions
        self.nb_actions = 0
        # for TD learning with fuzzy states:
        self.Q = np.zeros([0,0,2]) # reward value learned by association ~ like QLearning with TD
        # used to compute V:
        self.R = np.zeros([0,0]) # ---------- reward ---
        self.IR = np.zeros([0,0]) # ---------- intensity * reward ---
        self.n = np.zeros([0,0])
        self.matter = np.ones([0,2]) # importance of events

        #self.add_actions(["reason"])


    """ functions for updating and using list of activated cells """
    #--------------------------------------------------------------
    def add_activated(self, cell):
        if self.activateds:
            if len(self.activateds)==FIRE_TIME:
                for i in range(len(self.activateds)-1):
                    self.activateds[i] = self.activateds[i+1]
                self.activateds[-1] = cell
            else:
                self.activateds.append(cell)
        else:
            self.activateds.append(cell)

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

    def add_cells(self, cells_id):
        if isinstance(cells_id, list) or isinstance(cells_id, tuple):
            number = self.nb_cells
            for cell_id in cells_id:
                self.intensities.setdefault(cell_id,0)
                if cell_id not in self.cell_number:
                    self.cell_number[cell_id] = number
                    number += 1

                    new_counts = np.zeros([self.nb_actions,number, number])
                    new_counts[:,:self.nb_cells,:self.nb_cells] = self.counts
                    self.counts = new_counts

                    new_cor = np.zeros([self.nb_actions, number, number,2])
                    new_cor[:,:self.nb_cells,:self.nb_cells,:] = self.cor
                    self.cor = new_cor

                    new_matter = np.ones([number,2])
                    new_matter[:self.nb_cells,:] = self.matter
                    self.matter = new_matter

                    new_goals = np.zeros([number])
                    new_goals[:self.nb_cells] = self.goals
                    self.goals = new_goals

                    new_rewards = np.zeros([number,2])
                    new_rewards[:self.nb_cells,:] = self.rewards
                    self.rewards = new_rewards

                    new_Q = np.zeros([number, self.nb_actions,2])
                    new_Q[:self.nb_cells,:self.nb_actions,:] = self.Q
                    self.Q = new_Q

                    new_R = np.zeros([number, self.nb_actions])
                    new_R[:self.nb_cells,:self.nb_actions] = self.R
                    self.R = new_R

                    new_IR = np.zeros([number, self.nb_actions])
                    new_IR[:self.nb_cells,:self.nb_actions] = self.IR
                    self.IR = new_IR

                    new_n = np.zeros([number, self.nb_actions])
                    new_n[:self.nb_cells,:self.nb_actions] = self.n
                    self.n = new_n

                    self.nb_cells = number

    def add_actions(self, cells_id):
        if isinstance(cells_id, list) or isinstance(cells_id, tuple):
            self.add_cells(cells_id)
            number = self.nb_actions
            for cell_id in cells_id:
                if cell_id not in self.action_number:
                    self.action_number[cell_id] = number
                    number += 1

                    new_counts = np.zeros([number, self.nb_cells, self.nb_cells])
                    new_counts[:self.nb_actions,:,:] = self.counts
                    self.counts = new_counts

                    new_cor = np.zeros([number, self.nb_cells, self.nb_cells,2])
                    new_cor[:self.nb_actions,:,:,:] = self.cor
                    self.cor = new_cor

                    new_Q = np.zeros([self.nb_cells, number,2])
                    new_Q[:,:self.nb_actions,:] = self.Q
                    self.Q = new_Q

                    new_R = np.zeros([self.nb_cells, number])
                    new_R[:,:self.nb_actions] = self.R
                    self.R = new_R

                    new_IR = np.zeros([self.nb_cells, number])
                    new_IR[:,:self.nb_actions] = self.IR
                    self.IR = new_IR

                    new_n = np.zeros([self.nb_cells, number])
                    new_n[:,:self.nb_actions] = self.n
                    self.n = new_n

                    self.nb_actions = number

    def set_rewards(self, goals):
        for goal in goals:
            cell_id = goal[0]
            value = goal[1]
            reward = goal[2]
            if cell_id not in self.cell_number:
                self.add_cells([cell_id])
            if value>1:
                value=1.
            if value<-1:
                value=-1.
            self.rewards[self.cell_number[cell_id],value>0] = reward


    def update(self, percepts=None):

        # FIND THE NEXT ACTIVATED:
        elligibles = {}
        new_intensities = {}

        # REASONING:
        #===========
        #if self.action=="reason": # (no reasoning/thought for the moment)
        if self.activateds:

            # following cor: (no preference for different delays for the moment)
            #delay = 0
            #for activated in self.activateds:
            # TODO: loop on all previous activated cells taking account delayed causality

            intensity = self.old_intensities[-1]
            activated = self.activateds[-1]

            noise = np.random.rand(len(self.counts[0,0,:]))/100.
            proba_of_sons = self.counts[self.action_number[self.action],self.cell_number[activated],:]+noise
            next_num = np.argmax(proba_of_sons)
            proba = np.max(proba_of_sons)
            next_id = self.cell_number.inv[next_num]
            next_intensity = self.cor[self.action_number[self.action],self.cell_number[activated],next_num,intensity]

            if next_id not in percepts:
                elligibles.setdefault(next_id,0)
                elligibles[next_id] = np.abs(self.matter[next_num,next_intensity>0])*proba

                new_intensities.setdefault(next_id,0)
                new_intensities[next_id] = next_intensity
                if new_intensities[next_id]>1.:
                    new_intensities[next_id]=1.
                if new_intensities[next_id]<-1.:
                    new_intensities[next_id] = -1.
                #delay += 1

            # because recently activated:
            for activated in self.activateds[:-1]:
                elligibles.setdefault(activated,0)
                elligibles[activated] += 0 # arbitrary value, should be a global value

                #new_intensities.setdefault(activated,0)
                #new_intensities[activated] += 0.5*(1 - self.intensities[activated])

        # PERCEPTION:
        #============
        tot_reward = 0
        if percepts:
            for percept in percepts:

                percept_id = percept[0]
                percept_val = percept[1]
                percept_num = self.cell_number[percept_id]

                self.intensities[percept_id] = percept_val

                if self.action:
                    tot_reward += self.rewards[percept_num,percept_val>0]*np.abs(self.old_intensities[-1])

                elligibles.setdefault(percept_id,0)
                elligibles[percept_id] += np.exp(THETA2*np.abs(self.matter[self.cell_number[percept_id],percept_val>0]))

                if self.action:
                    #if not self.thinking[-1]:
                    father = self.activateds[-1]
                    son = percept_id
                    intensity_father = self.old_intensities[-1]
                    intensity_son = percept_val
                    action = self.action
                    self.reinforce(father,son,action,intensity_father,intensity_son)

        # stochastic election of incoming active cell:
        next_activated = random_pull_dict(elligibles)

        # new intensities:
        for cell in new_intensities:
            if cell not in self.modifieds:
                self.intensities[cell] = new_intensities[cell]
                self.modifieds.add(cell)

        # TODO: loop on the previous percept in past to make reinforcement with delay

        # action learning:
        if self.action:
            self.learn(next_activated,tot_reward)

        # new activated cell
        self.add_activated(next_activated)
        self.add_intensity(self.intensities[next_activated])

        # make decision:
        return self.decision()


    def reinforce(self, cell1, cell2, action, I1, I2):
        num_cell1 = self.cell_number[cell1]
        num_cell2 = self.cell_number[cell2]
        num_act = self.action_number[action]

        s = np.sum(self.counts[num_act,num_cell1])
        v = self.counts[num_act,num_cell1,num_cell2]
        self.counts[num_act,num_cell1,:] *= s/(s+1.)
        self.counts[num_act,num_cell1,num_cell2] = (s*v+1.)/(s+1.)
        #self.counts[self.counts[num_act,num_cell1]<MIN] = 0.

        self.cor[num_act,num_cell1,num_cell2,I1>0] = 0.9*self.cor[num_act][num_cell1][num_cell2][I1>0] + 0.1*I2


    def decision(self):
        state = self.cell_number[self.activateds[-1]]
        I = self.old_intensities[-1]
        # TODO exploration based on convergence/difficulty to reach a state
        values = self.Q[state,:,I>0]*np.abs(I)+np.random.rand(len(self.Q[state,:,I>0]))/10
        choice , expect = softmax(values)
        #choice = np.argmax(values)
        #expect = np.max(values)
        self.expected = expect
        self.action = self.action_number.inv[choice]
        return self.action


    def learn(self,new_activated,reward):
        if self.activateds and self.action:

            # last state:
            action = self.action_number[self.action]
            last_state = self.cell_number[self.activateds[-1]]
            last_intensity = self.old_intensities[-1]

            # new state:
            new_state = self.cell_number[new_activated]
            new_intensity = self.intensities[new_activated]
            new_values = self.Q[new_state,:,new_intensity>0]*np.abs(new_intensity)
            reach = np.max(new_values)

            # TD learning:
            # TODO : (using EMA instead of online average could be better for world with changing rules)
            TD = ( reward + DISCOUNT*reach - self.expected )
            n = self.n[last_state][action]+1.

            self.Q[last_state,action,last_intensity>0] = (n*self.Q[last_state,action,last_intensity>0] + TD)/(n+1.)
            self.n[last_state][action] += 1.
            self.matter[new_state,new_intensity>0] = (n*(self.matter[new_state,new_intensity>0]) + TD)/(n+1.)

            print "last "+str(self.activateds[-1])+" "+str(last_intensity)
            print "act "+ str(self.action)
            print "new "+str(new_activated)+" "+str(new_intensity)
            print "rew "+str(TD)
            print "======================"
