#!/usr/bin/env python
# coding: utf-8

"""
library of functions/algorithms to build, update and compaire models of different agents built by a robot.
"""

import numpy as np
import random
import operator
from bidict import bidict


""" GLOBAL PARAMETERS """
# hebbian learning:
STIFFNESS = 3 # how I expect the most likely event
FIRE_TIME = 10 # time a cell is activated
COUNT_MAX = 0. # plasticity
THRESHOLD = 2. # start to learn when something is recurrent
MIN = 0.00001 # minimal transition probability
GAMMA = 0.1 # time discount for learning

# reinforcement learning:
THETA = 1.5 # exponent for softmax pulling
DISCOUNT = 0.50 # discount for the impact of futur on the temporal diff algo
ALPHA = 0.5 # learning rate imediate (R)
BETA = 0.05 # learning rate indirect (Q)

""" functions for spiking cascade following distribution of weights"""
#--------------------------------------------------------------------
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
        expo = np.exp(distribution*THETA)
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
        self.intensities = {} # list of cell's intensity between -1 and 1 (intensity or truth)
        self.nb_cells = 0
        self.activateds = [] # list of activated cells, the first is the most recently activated (contains by default the cell encoding the empty concept)
        self.perceiveds = [] # are cells activated because perception (1) or because reasoning (0) ?
        self.old_intensities = []
        self.modifieds = set() # for each input from exterior (percept or percept) cell intensities are modified once
                               # it makes the differrence between the flow of thought and real perception
        self.cell_number = bidict() # each cell is numeroted {cell_id <--> cell_number}

        # hebbian learning:
        #------------------
        self.counts = np.zeros([0,0,0]) # count the close activations for hebbian learning
        self.weights = np.zeros([0,0,0]) # weights of connexion between cells for different possible actions

        # reinforcement learning:
        #------------------------
        self.action = None
        self.expected = 0
        self.goals = np.zeros([0]) # goal value (1 or -1) for each cells
        self.rewards = np.zeros([0]) # reward associated with goals (0 if no objective)
        self.action_number = bidict() # set of cells encoding actions
        self.nb_actions = 0
        # for actor-critic decision making :
        self.V = np.zeros([0,0]) # nb_cells*nb_actions := optimal intensity of a cell to use the action
        self.Q = np.ones([0,0]) # reward value learned by association ~ like QLearning with TD
        # used to compute V:
        self.I = np.zeros([0,0]) # cumulative value of intensities while chosing action with this event
        self.R = np.zeros([0,0]) # ---------- reward ---
        self.IR = np.zeros([0,0]) # ---------- intensity * reward ---
        self.Rmin = np.zeros([0,0]) # minimum reward obtained for couple (event,action)
        self.n = np.zeros([0,0])
        self.matter = np.ones([0]) # importance of events

        # network:= [intensities , counts, times, weights]



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

                    new_weights = np.zeros([self.nb_actions, number, number])
                    new_weights[:,:self.nb_cells,:self.nb_cells] = self.weights
                    self.weights = new_weights

                    new_matter = np.ones([number])
                    new_matter[:self.nb_cells] = self.matter
                    self.matter = new_matter

                    new_goals = np.zeros([number])
                    new_goals[:self.nb_cells] = self.goals
                    self.goals = new_goals

                    new_rewards = np.zeros([number])
                    new_rewards[:self.nb_cells] = self.rewards
                    self.rewards = new_rewards

                    new_v = np.zeros([number, self.nb_actions])
                    new_v[:self.nb_cells,:self.nb_actions] = self.V
                    self.V = new_v

                    new_q = np.ones([number, self.nb_actions])
                    new_q[:self.nb_cells,:self.nb_actions] = self.Q
                    self.Q = new_q

                    new_I = np.zeros([number, self.nb_actions])
                    new_I[:self.nb_cells,:self.nb_actions] = self.I
                    self.I = new_I

                    new_R = np.zeros([number, self.nb_actions])
                    new_R[:self.nb_cells,:self.nb_actions] = self.R
                    self.R = new_R

                    new_IR = np.zeros([number, self.nb_actions])
                    new_IR[:self.nb_cells,:self.nb_actions] = self.IR
                    self.IR = new_IR

                    new_Rmin = np.zeros([number, self.nb_actions])
                    new_Rmin[:self.nb_cells,:self.nb_actions] = self.Rmin
                    self.Rmin = new_Rmin

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

                    new_weights = np.zeros([number, self.nb_cells, self.nb_cells])
                    new_weights[:self.nb_actions,:,:] = self.weights
                    self.weights = new_weights

                    new_V = np.zeros([self.nb_cells, number])
                    new_V[:,:self.nb_actions] = self.V
                    self.V = new_V

                    new_Q = np.ones([self.nb_cells, number])
                    new_Q[:,:self.nb_actions] = self.Q
                    self.Q = new_Q

                    new_I = np.zeros([self.nb_cells, number])
                    new_I[:,:self.nb_actions] = self.I
                    self.I = new_I

                    new_R = np.zeros([self.nb_cells, number])
                    new_R[:,:self.nb_actions] = self.R
                    self.R = new_R

                    new_IR = np.zeros([self.nb_cells, number])
                    new_IR[:,:self.nb_actions] = self.IR
                    self.IR = new_IR

                    new_Rmin = np.zeros([self.nb_cells, number])
                    new_Rmin[:,:self.nb_actions] = self.Rmin
                    self.Rmin = new_Rmin

                    new_n = np.zeros([self.nb_cells, number])
                    new_n[:,:self.nb_actions] = self.n
                    self.n = new_n

                    self.nb_actions = number

    def set_goal(self, goals):
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
            self.goals[self.cell_number[cell_id]] = value
            self.rewards[self.cell_number[cell_id]] = reward


    def update(self, percepts=None):

        #   # FIND THE NEXT ACTIVATED:
        elligibles = {}
        new_intensities = {}

        #   # following weights: (no preference for different delays for the moment)
        #   delay = 0
        #   for activated in self.activateds:

        #       intensity = self.intensities[activated]
        #       proba_of_sons = self.counts[delay][self.cell_number[activated]][:]
        #       next_num,_ = random_pull_list(proba_of_sons)
        #       next_id = self.cell_number.inv[next_num]
        #       strength = intensity*self.counts[delay][self.cell_number[activated]][next_num]

        #       elligibles.setdefault(next_id,0)
        #       elligibles[next_id] += np.abs(strength)

        #       new_intensities.setdefault(next_id,0)
        #       new_intensities[next_id] += strength - np.abs(strength)*self.intensities[next_id]
        #       if new_intensities[next_id]>1.:
        #           new_intensities[next_id]=1.
        #       if new_intensities[next_id]<-1.:
        #           new_intensities[next_id] = -1.
        #       delay += 1

        #   # because recently activated:
        #   for activated in self.activateds[:-1]:
        #       elligibles.setdefault(activated,0)
        #       elligibles[activated] += 0.5 # arbitrary value, should be a global value

        #       #new_intensities.setdefault(activated,0)
        #       #new_intensities[activated] += 0.5*(1 - self.intensities[activated])

        #   # because perception:
        #   if percepts: # percept must be associated with a new intensity !
        #       for percept in percepts:

        #           percept_id = percept[0]
        #           percept_val = percept[1]

        #           self.intensities[percept_id] = percept_val

        #           elligibles.setdefault(percept_id,0)
        #           elligibles[percept_id] += 1. # arbitrary ~ how I trust my perception

        #           if self.perceiveds:
        #               if self.perceiveds[-1]>0:
        #                   activated = self.activateds[-1]
        #                   correlation = self.intensities[percept_id] * self.intensities[activated]
        #                   if percept_id != activated:
        #                       self.reinforce(activated,percept_id,0,correlation)


        if percepts:
            for percept in percepts:

                percept_id = percept[0]
                percept_val = percept[1]

                self.intensities[percept_id] = percept_val

                elligibles.setdefault(percept_id,0)
                elligibles[percept_id] += np.exp(THETA*self.matter[self.cell_number[percept_id]])

                if self.perceiveds and self.action:
                    if self.perceiveds[-1]>0:
                        activated = self.activateds[-1]
                        action = self.action
                        correlation = self.intensities[percept_id] * self.intensities[activated]
                        #if percept_id != activated:
                        self.reinforce(activated,percept_id,correlation,action)

                # self.add_perceived(1.) that can't be here !!!

        # stochastic election of incoming active cell:
        next_activated = random_pull_dict(elligibles)

        # hebbian reinforcement
        #   test = False
        #   if percepts:
        #       test = test or (next_activated in np.array(percepts))
        #   if percepts:
        #       test = test or (next_activated in np.array(percepts))
        #   if test:
        #       self.modifieds = set()
        #       delay = 0
        #       for i in range(len(self.activateds)):
        #           activated = self.activateds[i]
        #           perceived = self.perceiveds[i]
        #           weakness = GAMMA**delay
        #           correlation = self.intensities[next_activated] * self.intensities[activated] * (weakness+perceived-weakness*perceived)
        #           if next_activated != activated:
        #               self.reinforce(activated,next_activated,delay,correlation)
        #           delay += 1

        #       self.add_perceived(1.)
        #       #self.reward()

        #   if not test:
        #       self.add_perceived(0.)

        #       if next_activated in self.action_number: # I imagine a strategy
        #           if self.activateds:
        #               self.strategy.append([self.activateds[-1],next_activated]) # (state, action)

        # self.reward(self.activateds[-1],next_activated,action)

        # action learning:
        if self.action:
            self.reward(next_activated,self.intensities[next_activated])

        # new activated cell
        self.add_activated(next_activated)
        self.add_intensity(self.intensities[next_activated])

        # new intensities:
        for cell in new_intensities:
            if cell not in self.modifieds:
                self.intensities[cell] = new_intensities[cell]
                self.modifieds.add(cell)

        # make decision:
        return self.decision()


    def reinforce(self, cell1, cell2, correlation, action):

        global THRESHOLD
        global COUNT_MAX
        global FORGET_RATE

        #print self.cell_number

        num_cell1 = self.cell_number[cell1]
        num_cell2 = self.cell_number[cell2]
        num_act = self.action_number[action]

        #n = self.counts[num_cell1][num_cell2]
        #t = self.times[num_cell1][num_cell2]

        s = np.sum(self.counts[num_act][num_cell1])
        v = self.counts[num_act][num_cell1][num_cell2]
        self.counts[num_act][num_cell1][:] *= s/(s+1.)
        self.counts[num_act][num_cell1][num_cell2] = (s*v+1.)/(s+1.)
        self.counts[num_act][num_cell1][self.counts[num_act][num_cell1]<MIN] = 0.

        if self.counts[num_act][num_cell1][num_cell2]>0: 
            self.weights[num_act][num_cell1][num_cell2] += correlation - np.abs(correlation)*self.weights[num_act][num_cell1][num_cell2]
            if self.weights[num_act][num_cell1][num_cell2]>1:
                self.weights[num_act][num_cell1][num_cell2]=1.
            if self.weights[num_act][num_cell1][num_cell2]<-1:
                self.weights[num_act][num_cell1][num_cell2]=-1.


    def decision(self):
        state = self.cell_number[self.activateds[-1]]
        values = self.Q[state]*(1-np.abs(self.V[state]-self.intensities[self.activateds[-1]]))
        choice , expect = softmax(values)
        self.expected = expect
        self.action = self.action_number.inv[choice]
        return self.action

    def reward(self,new_activated,new_intensity):
        if self.activateds and self.action:

            # last state:
            action = self.action_number[self.action]
            last_state = self.cell_number[self.activateds[-1]]
            last_intensity = self.old_intensities[-1]

            # new state:
            new_state = self.cell_number[new_activated]
            reward = self.rewards[new_state]*(1.-np.abs(self.goals[new_state]-new_intensity))
            new_values = self.Q[new_state]*(1-np.abs(self.V[new_state]-new_intensity))
            reach = np.max(new_values)

            #print "----"
            #print self.activateds[-1]
            #print last_intensity
            #print self.action

            #print "'------"
            #print new_activated
            #print new_intensity
            #print reward

            TD = reward + DISCOUNT*reach - self.expected
            R = ALPHA*reward
            #print TD

            n = self.n[last_state][action]+1.

            #self.Q[last_state][action] += BETA*TD
            self.Q[last_state][action] = 1+(n*self.Q[last_state][action]+TD)/(n+1.)

            self.I[last_state][action] += last_intensity
            self.R[last_state][action] += R
            self.IR[last_state][action] += R*last_intensity
            self.n[last_state][action] += 1
            if R < self.Rmin[last_state][action]:
                self.Rmin[last_state][action] = R

            I = self.I[last_state][action]
            R = self.R[last_state][action]
            IR = self.IR[last_state][action]
            Rmin = self.Rmin[last_state][action]
            n = self.n[last_state][action]

            self.V[last_state][action] = (IR - I*Rmin)/(R - n*Rmin + 0.001)
            #print self.V[last_state][action]

            #self.matter[new_state] += BETA*np.abs(TD)
            self.matter[new_state] = 1+(n*self.matter[new_state] + np.abs(TD))/(n+1.)



