#!/usr/bin/env python
# coding: utf-8

"""
library of functions/algorithms to build, update and compaire models of different agents built by a robot.
"""

import numpy as np


""" functions for spiking cascade following distribution of weights"""
#--------------------------------------------------------------------
def stochastic_compare_couples(c1, c2):
     a,b = c1
     c,d = c2
     s = np.abs(b) + np.abs(d)
     r = random.uniform(0,s)
     return 1 if r>np.abs(b) else -1

def stochastic_compare_dict(x, y):
     s = np.abs(x) + np.abs(y)
     r = random.uniform(0,s)
     return 1 if r>np.abs(x) else -1

def random_pull_couples(distribution): # dist. is a set of coulpes
     if distribution :
         sorted_couples = sorted(distribution,cmp=stochastic_compare_couples)
         return sorted_couples[0][0], np.abs(sorted_couples[0][1])
     else:
         return None

def random_pull_dict(distribution): # dist. is a dictionnary key->value
     if distribution :
         sorted_couples = sorted(distribution.items(), key=operator.itemgetter(1), cmp=stochastic_compare_dict)
         return sorted_couples[0][0], np.abs(sorted_couples[0][1])
     else:
         return None


""" functions for updating and using list of activated cells """
#--------------------------------------------------------------
def add_activated(activated, cell):
    for i in range(len(activated)-1):
        activated[i] = activated[i+1]
    activated[-1] = cell


""" object Model """
#-------------------
class Model:
    """ an object 'Model' representing hebbian-inspired network that encode dynamics between events learned by an agent."""
    def __init__(self, network=None, current=None):

        # DEFAULT:
        self.intensities = {} # list of cell's intensity between 0 and 1 (intensity or truth)
        # self.current = "" # the current activated cell (empty event by default)
        self.weights = [{}] # list of dict of cells->{(following cells,weight,...} between -1 and 1
        # weights[i] is the weigths for transmission after time i
        self.weights[0][""] = {}
        """
        self.times = {} # list of cell->(following cells,time for next activation = integer)
        self.times[""] = {}
        """
        self.activateds = [""] # list of activated cells, the first is the most recently activated (contains by default the empty event)

        #self.queue = [] # queue of following (cell,time) to be activated depending on the time

        # network:= [event , intensity , followers]
        # followers:= [name , weight , time]

        # FILLING:
        if network:

            for link in network:
                event = link[0]
                intensity = link[1]
                if intensity>1:
                    intensity = 1
                if intensity<0:
                    intensity = 0
                self.intensities[event] = intensity
                followers = link[2]

                for time in range(FIRE_TIME):
                    time_event = {}
                    time_event.setdefault(event,{})
                    time_event[""][event] = 1

                    for follower in followers:
                        name = follower[0]
                        weight = follower[1]
                        if weight>1:
                            weight = 1
                        if weight<-1:
                            weight = -1
                        self.intensity.setdefault(name,0.5)
                        time_event[event][name] = weight

                    self.weights.append(time_event)

        if activateds:
            self.activateds = activateds

    def update(self):

        # FIND THE NEXT ACTIVATED:
        elligibles = {}

        # By hebbian rule: (no delay for the moment)
        time = 0
        for activated in self.activateds:
            followers = self.weights[time][activated]
            next_id, abs_weight = random_pull_couples(followers)
            elligibles.setdefault(next_id,0)
            elligibles[next_id] += abs_weight
            time += 1

        # because recently activated:
        for activated in self.activated[-1]:
            elligibles.setdefault(activated,0)
            elligibles[activated] += 0.5 # arbitrary value, should be a global value

        # because perception:
        if percepts:
            for percept in percepts:
                elligibles.setdefault(percept,0)
                elligibles[percept] += 1. # arbitrary

        # because very important percept:
        if reflexes:
            for reflex in reflexes:
                elligibles.setdefault(reflex,0)
                elligibles[reflex] += 10. # arbitrary

        # stochastic election:
        next_activated,_ = random_pull_dict(elligibles)
        add_activated(self.activateds,next_activated)

        # new intensity:

        # hebbian reinforcement:





