#!/usr/bin/env python
# coding: utf-8

"""
library of functions/algorithms to build, update and compaire models of different agents built by a robot.
"""

import numpy as np

class Model:
""" an object 'Model' representing neuronal-inspired network that encode dynamics between events learned by an agent."""
    def __init__(self, events=None):
        self.intensities = {} # list of cell's intensity between 0 and 1 (intensity or truth)
        self.current = "" # the current activated cell (empty event by default)
        self.weights = {} # list of cell->(following cells,weigt)
        self.times = {} # list of cell->(following cells,time for next activation = integer)
        self.queue = [] # queue of following (cell,time) to be activated depending on the time
        if events:
            for event in events:

