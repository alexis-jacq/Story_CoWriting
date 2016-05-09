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
STIFFNESS = 3
FIRE_TIME = 5
COUNT_MAX = 10.
THRESHOLD = 3.
FORGET_RATE = 0.05

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
        self.modifieds = set() # for each input from exterior (percept or reflex) cell intensities are modified once
                               # it makes the differrence between the flow of thought and real perception
        self.cell_number = bidict() # each cell is numeroted {cell_id <--> cell_number}
        self.counts = np.zeros([0,0]) # count the close activations for hebbian learning
        self.times = np.zeros([0,0]) # count the average delay between two close activations
        self.weights = np.zeros([FIRE_TIME,0,0]) # weights of connexion between cells for different possible (interger) delays

        # network:= [intensities , counts, times, weights]

        # FILLING:
        if network:
            self.intensities = network[0]
            self.nb_cells = len(self.intensities)
            # and cell_number !

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


    def add_cells(self,cells_id):
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

            self.nb_cells = number


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

                #new_intensities.setdefault(percept_id,0)
                #new_intensities[percept_id] += 1*(1 - float(self.intensities[percept_id]))

        # because very important percept:
        if reflexes:
            for reflex in reflexes:

                reflex_id = reflex[0]
                reflex_val = reflex[1]

                self.intensities[reflex_id] = reflex_val

                elligibles.setdefault(reflex_id,0)
                elligibles[reflex_id] += 10. # arbitrary

                #new_intensities.setdefault(reflex_id,0)
                #new_intensities[reflex_id] += 1*(1 - self.intensities[reflex_id]) # 1 is the max in order to stay btween -1 and 1

        # stochastic election and hebbian reinforcement:
        next_activated = random_pull_dict(elligibles)
        test = False
        if reflexes:
            test = test or (next_activated in np.array(reflexes))
        if percepts:
            test = test or (next_activated in np.array(percepts))
        if test:
            self.modifieds = set()
            delay = 0
            for activated in self.activateds:
                correlation = self.intensities[next_activated] * self.intensities[activated]
                if next_activated != activated:
                    self.reinforce(activated,next_activated,delay,correlation)
                delay += 1
        self.add_activated(next_activated)

        # new intensities:
        for cell in new_intensities:
            if cell not in self.modifieds:
                self.intensities[cell] = new_intensities[cell]
                self.modifieds.add(cell)


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






