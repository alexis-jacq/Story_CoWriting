#!/usr/bin/env python
# coding: utf-8

"""
library of functions/algorithms to build, update and compaire models of different agents built by a robot.
"""

import numpy as np

class Model:
""" an object 'Model' representing hebbian-inspired network that encode dynamics between events learned by an agent."""
    def __init__(self, network=None, current=None):

        # DEFAULT:
        self.intensities = {} # list of cell's intensity between 0 and 1 (intensity or truth)
        self.current = "" # the current activated cell (empty event by default)
        self.weights = {} # list of cell->(following cells,weigt) between -1 and 1
        self.weights[""] = {}
        self.times = {} # list of cell->(following cells,time for next activation = integer)
        self.times[""] = {}

        #self.queue = [] # queue of following (cell,time) to be activated depending on the time

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
                self.weights.setdefault(event,{})
                self.times.setdefault(event,{})
                self.weights[""][event] = 1
                self.times[""][event] = 1

                for follower in followers:
                    name = follower[0]
                    weight = follower[1]
                    if weight>1:
                        weight = 1
                    if weight<-1:
                        weight = -1
                    time = np.abs(np.floor(follower[2])+1)
                    self.intensity.setdefault(name,0.5)
                    self.weights[event][name] = weight
                    self.times[event][name] = time

        if current:
            self.current = current




