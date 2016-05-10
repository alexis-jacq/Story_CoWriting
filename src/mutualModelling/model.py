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
STIFFNESS = 3 # how I expect the most likely event
FIRE_TIME = 5 # time a cell is activated
COUNT_MAX = 10. # plasticity
THRESHOLD = 3. # start to learn when something is recurrent
FORGET_RATE = 0.01 # kill the noise
GAMMA = 0.5 # time discount for learning

""" functions for spiking cascade following distribution of weights"""
#--------------------------------------------------------------------
def stochastic_compare_couples(c1, c2):
    global STIFFNESS
    a,b = c1
    c,d = c2
    s = np.abs(b)**STIFFNESS + np.abs(d)**STIFFNESS
    r = random.uniform(0,s)
    return 1 if r>np.abs(b)**STIFFNESS else -1

def stochastic_compare(x, y):
    s = np.abs(x) + np.abs(y)
    r = random.uniform(0,s)
    return 1 if r>np.abs(x) else -1

def random_pull_list(distribution): # dist. is a list (np.array) of values
    if list(distribution):
        couples = zip(range(len(distribution)),distribution)
        sorted_couples = sorted(couples,cmp=stochastic_compare_couples)
        return sorted_couples[0][0], sorted_couples[0][1]
    else:
        return None

def random_pull_dict(distribution): # dist. is a dictionnary key->value
     if distribution:
         sorted_dist = sorted(distribution.items(), key=operator.itemgetter(1), cmp=stochastic_compare)
         return sorted_dist[0][0]
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
        self.modifieds = set() # for each input from exterior (percept or reflex) cell intensities are modified once
                               # it makes the differrence between the flow of thought and real perception
        self.cell_number = bidict() # each cell is numeroted {cell_id <--> cell_number}

        # hebbian learning:
        #------------------
        self.counts = np.zeros([0,0]) # count the close activations for hebbian learning
        self.times = np.zeros([0,0]) # count the average delay between two close activations
        self.weights = np.zeros([FIRE_TIME,0,0]) # weights of connexion between cells for different possible (interger) delays

        # reinforcement learning:
        #------------------------
        self.goals = np.zeros([0]) # goal value (1 or -1) for each cells (0 mean no objective)
        self.actions = set() # set of cells encoding actions
        self.nb_actions = 0
        self.strategy = [] # a strategy is a list of (state,action) where state is the last activated cell up to a final reward
        # for actor-critic decision making :
        self.value_map = np.zeros([0,0]) # nb_cells*nb_actions := value given for couple (action,event)
        self.proba_map = np.zeros([0,0]) # nb_cells*nb_actions := proba to chose the action given an event

        # network:= [intensities , counts, times, weights]

        # FILLING:
        if network:
            self.intensities = network[0]
            self.nb_cells = len(self.intensities)
            # and cell_number !
            # and action 
            # and ...

            counts = network[1]
            times = network[2]
            weights = network[3]

            if (counts.shape==times.shape) and (times.shape==weights.shape) and (times.shape==(self.nb_cells,self.nb_cells)):
                self.counts = counts
                self.times = times
                self.weights = weights
            else:
                print "Error of dimension in the loaded network !"
                print "Default = np.zeros array"
                self.counts = np.zeros([self.nb_cells,self.nb_cells])
                self.times = np.zeros([self.nb_cells,self.nb_cells])
                self.weights = np.zeros([FIRE_TIME,self.nb_cells,self.nb_cells])

        if activateds:
            self.activateds = activateds

        if modifieds:
            self.modifieds = modifieds

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

    def add_perceived(self, val):
        if self.activateds:
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

                    new_counts = np.zeros([number, number])
                    new_counts[:self.nb_cells,:self.nb_cells] = self.counts
                    self.counts = new_counts

                    new_times = np.zeros([number, number])
                    new_times[:self.nb_cells,:self.nb_cells] = self.times
                    self.times = new_times

                    new_weights = np.zeros([FIRE_TIME, number, number])
                    new_weights[:,:self.nb_cells,:self.nb_cells] = self.weights
                    self.weights = new_weights

                    new_goals = np.zeros([number])
                    new_goals[:self.nb_cells] = self.goals
                    self.goals = new_goals

                    self.nb_cells = number

    def add_actions(self, cells_id):
        if isinstance(cells_id, list) or isinstance(cells_id, tuple):
            self.add_cells(cells_id)
            number = self.nb_actions
            for cell_id in cells_id:
                if cell_id not in self.actions:
                    number += 1

                    new_vmap = np.zeros([number, number])
                    new_vmap[:,:self.nb_actions] = self.value_map
                    self.value_map = new_vmap

                    new_pmap = np.zeros([number, number])
                    new_pmap[:,:self.nb_actions] = self.proba_map
                    self.proba_map = new_pmap

                    self.nb_actions = number

    def set_goal(self, goals):
        for goal in goals:
            cell_id = goal[0]
            value = goal[1]
            if cell_id not in self.cell_number:
                self.add_cells([cell_id])
            if value>1:
                value=1.
            if value<-1:
                value=-1.
            self.goals[cell_id] = value


    def update(self, percepts=None, reflexes=None):

        # FIND THE NEXT ACTIVATED:
        elligibles = {}
        new_intensities = {}

        # following weights: (no preference for different delays for the moment)
        delay = 0
        for activated in self.activateds:

            intensity = self.intensities[activated]
            weights_to_sons = self.weights[delay][self.cell_number[activated]][:]
            next_num, strength = random_pull_list(intensity*weights_to_sons)
            next_id = self.cell_number.inv[next_num]

            elligibles.setdefault(next_id,0)
            elligibles[next_id] += np.abs(strength)

            new_intensities.setdefault(next_id,0)
            new_intensities[next_id] += strength - np.abs(strength)*self.intensities[next_id]
            if new_intensities[next_id]>1.:
                new_intensities[next_id]=1.
            if new_intensities[next_id]<-1.:
                new_intensities[next_id] = -1.
            delay += 1

        # because recently activated:
        for activated in self.activateds[:-1]:
            elligibles.setdefault(activated,0)
            elligibles[activated] += 0.5 # arbitrary value, should be a global value

            #new_intensities.setdefault(activated,0)
            #new_intensities[activated] += 0.5*(1 - self.intensities[activated])

        # because perception:
        if percepts: # percept must be associated with a new intensity !
            for percept in percepts:

                percept_id = percept[0]
                percept_val = percept[1]

                self.intensities[percept_id] = percept_val

                elligibles.setdefault(percept_id,0)
                elligibles[percept_id] += 1. # arbitrary ~ how I trust my perception

                if self.perceiveds:
                    if self.perceiveds[-1]>0:
                        activated = self.activateds[-1]
                        correlation = self.intensities[percept_id] * self.intensities[activated]
                        if percept_id != activated:
                            self.reinforce(activated,percept_id,0,correlation)

        # because very important percept:
        if reflexes:
            for reflex in reflexes:

                reflex_id = reflex[0]
                reflex_val = reflex[1]

                self.intensities[reflex_id] = reflex_val

                elligibles.setdefault(reflex_id,0)
                elligibles[reflex_id] += 10. # arbitrary

                if self.perceiveds:
                    if self.perceiveds[-1]>0:
                        activated = self.activateds[-1]
                        correlation = self.intensities[reflex_id] * self.intensities[activated]
                        if reflex_id != activated:
                            self.reinforce(activated,reflex_id,0,correlation)

        # stochastic election of incoming active cell:
        next_activated = random_pull_dict(elligibles)

        # hebbian reinforcement
        test = False
        if reflexes:
            test = test or (next_activated in np.array(reflexes))
        if percepts:
            test = test or (next_activated in np.array(percepts))
        if test:
            self.modifieds = set()
            delay = 0
            for i in range(len(self.activateds)):
                activated = self.activateds[i]
                perceived = self.perceiveds[i]
                weakness = GAMMA**delay
                correlation = self.intensities[next_activated] * self.intensities[activated] * (weakness+perceived-weakness*perceived)
                if next_activated != activated:
                    self.reinforce(activated,next_activated,delay,correlation)
                delay += 1

            self.add_perceived(1.)
            self.reward()

        if not test:
            self.add_perceived(0.)

            if next_activated in self.actions: # I imagine a strategy
                if self.activateds:
                    self.strategy.append([self.activateds[-1],next_activated])

        # new activated cell
        self.add_activated(next_activated)

        # action learning:
        self.reward()

        # new intensities:
        for cell in new_intensities:
            if cell not in self.modifieds:
                self.intensities[cell] = new_intensities[cell]
                self.modifieds.add(cell)

        # make decision:
        return self.decision()


    def reinforce(self, cell1, cell2, delay, correlation):

        global THRESHOLD
        global COUNT_MAX
        global FORGET_RATE

        #print self.cell_number

        num_cell1 = self.cell_number[cell1]
        num_cell2 = self.cell_number[cell2]

        n = self.counts[num_cell1][num_cell2]
        t = self.times[num_cell1][num_cell2]

        z = FORGET_RATE*np.sum(self.counts[num_cell1][:])
        self.counts[num_cell1][:] -= z
        self.counts[num_cell1][self.counts[num_cell1]<0] = 0.

        if self.counts[num_cell1][num_cell2]+z+1. > COUNT_MAX:
            self.counts[num_cell1][num_cell2] += z
        else:
            self.counts[num_cell1][num_cell2] += z+1.

        self.times[num_cell1][num_cell2] = (n*t + delay)/(n+1.) # iterative computation of average delay

        if self.counts[num_cell1][num_cell2]>=THRESHOLD: # thist trheshold could be smaller than the other one
            self.weights[delay][num_cell1][num_cell2] += correlation - np.abs(correlation)*self.weights[delay][num_cell1][num_cell2]
            if self.weights[delay][num_cell1][num_cell2]>1:
                self.weights[delay][num_cell1][num_cell2]=1.
            if self.weights[delay][num_cell1][num_cell2]<-1:
                self.weights[delay][num_cell1][num_cell2]=-1.


    def decision(self):

    def reward(self):


