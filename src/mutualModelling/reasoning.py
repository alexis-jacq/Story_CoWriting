#!/usr/bin/env python
# coding: utf-8

import numpy as np
import random
from mutualModelling import model
import matplotlib.pyplot as plt

class Agent:
    """agent able of first and 2nd mutual modelling reasoning"""
    def __init__(self):
        self.me = model.Model()

