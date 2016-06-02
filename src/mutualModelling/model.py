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
FIRE_TIME = 10 # time a cell is activated
ETA1 = 0.9 # for EMA of the correlation between intensity of signals
# if delayed hebbian: GAMMA = 0.1 # time discount for learning

# reinforcement learning:
#========================
THETA1 = 30#30 # chose action (exponent for softmax pulling
THETA2 = 30#20 # chose perception
ETA2 = 0.7
DISCOUNT = 0.9 # discount for the impact of futur on the temporal diff algo

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
    """ an object 'Model' representing hebbian-inspired network that encode dynamics between cells representing concepts learned by an agent."""
    def __init__(self, network=None, activateds=None, modifieds=None):

        global FIRE_TIME

        # DEFAULT:

        # cells encoding events:
        #-----------------------
        self.intensities = {} # list of cell's intensity between -1 and 1 (intensity or truth)
        self.nb_cells = 0
        self.activateds = [] # list of activated cells, the first is the most recently activated (contains by default the cell encoding the empty concept)
        self.old_intensities = []
        self.modifieds = set() # for each input from exterior (percept) cell intensities are modified once
                               # it makes the differrence between the flow of reasoning and real perception
        self.cell_number = bidict() # each cell is numeroted {cell_id <--> cell_number}

        # hebbian learning (world's causality):
        #--------------------------------------
        self.counts = np.zeros([0,0,0]) # count the close activations for hebbian learning
        self.cor = np.zeros([0,0,0,2]) # corelation with positive intensity

        # reinforcement learning (action):
        #---------------------------------
        self.action = None
        self.expected = 0
        self.rewards = np.zeros([0,2]) # reward associated with goals (0 if no objective)
        self.action_number = bidict() # set of cells encoding actions
        self.nb_actions = 0
        # for TD learning with fuzzy states:
        self.Q = np.zeros([0,0,2]) # reward value learned by association ~ like QLearning with TD
        self.n = np.zeros([0,0])
        self.matter = np.ones([0,2]) # importance of events

        #see "PERCEPTION" loop in "update" method:
        #self.add_actions(["force_reason"])


    """ functions for creating/updating/using models """
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

                    new_rewards = np.zeros([number,2])
                    new_rewards[:self.nb_cells,:] = self.rewards
                    self.rewards = new_rewards

                    new_Q = np.zeros([number, self.nb_actions,2])
                    new_Q[:self.nb_cells,:self.nb_actions,:] = self.Q
                    self.Q = new_Q

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
            self.rewards[self.cell_number[cell_id],int(value>0)] = reward


    def update(self, possible_actions=None, percepts=None):

        # FIND THE NEXT ACTIVATED:
        elligibles = {}
        new_intensities = {}

        """
        if not percepts and not self.activateds:
            self.add_cells([""])
            self.add_actions([""])
            self.add_activated("")
            self.add_intensity(0)
            self.action = ""
        """

        # REASONING:
        #===========
        if self.activateds and self.action: # (no reasoning/thought for the moment)

            # following cor: (no preference for different delays for the moment)
            #delay = 0
            #for activated in self.activateds:
            # TODO: loop on all previous activated cells taking account delayed causality

            intensity = self.old_intensities[-1]
            activated = self.activateds[-1]

            noise = np.random.rand(len(self.counts[0,0,:]))/1000.
            next_num=0
            proba=0
            next_id=""
            next_intensity=0
            if max(self.counts[self.action_number[self.action],self.cell_number[activated],:])>0:
                proba_of_sons = self.counts[self.action_number[self.action],self.cell_number[activated],:]+noise
                next_num = np.argmax(proba_of_sons)
                proba = np.max(proba_of_sons)
                next_id = self.cell_number.inv[next_num]
                next_intensity = self.cor[self.action_number[self.action],self.cell_number[activated],next_num,int(intensity>0)]
            else:
                proba_of_sons = self.counts[self.action_number[self.action],:,:]+noise
                next_num = np.argmax(proba_of_sons)%np.shape(proba_of_sons)[0]
                proba = np.max(proba_of_sons)
                next_id = self.cell_number.inv[next_num]
                next_intensity = self.cor[self.action_number[self.action],self.cell_number[activated],next_num,int(intensity>0)]
                #next_id = activated
                #next_intensity = intensity

            if (not percepts) or (next_id not in percepts):
                elligibles.setdefault(next_id,0)
                elligibles[next_id] = np.exp(THETA2*np.abs(self.matter[next_num,int(next_intensity>0)])*proba)

                new_intensities.setdefault(next_id,0)
                new_intensities[next_id] = next_intensity
                if new_intensities[next_id]>1.:
                    new_intensities[next_id]=1.
                if new_intensities[next_id]<-1.:
                    new_intensities[next_id] = -1.
                #delay += 1

        # PERCEPTION:
        #============
        # could add an action "force_reasoning" where the robot doesnot do the perception loop
        # like someone closing eyes in order to reason
        tot_reward = 0
        if percepts:
            for percept in percepts:
                if not (percept in self.cell_number):
                    self.add_cells([percept[0]])

                percept_id = percept[0]
                percept_val = percept[1]
                percept_num = self.cell_number[percept_id]

                self.intensities[percept_id] = percept_val

                if self.action and self.old_intensities:
                    tot_reward += self.rewards[percept_num,int(percept_val>0)]*np.abs(self.old_intensities[-1])

                elligibles.setdefault(percept_id,0)
                elligibles[percept_id] = np.exp(THETA2*np.abs(self.matter[self.cell_number[percept_id],int(percept_val>0)]))

                if self.action and self.activateds:
                    #if not self.thinking[-1]:
                    father = self.activateds[-1]
                    son = percept_id
                    intensity_father = self.old_intensities[-1]
                    intensity_son = percept_val
                    action = self.action
                    self.reinforce(father,son,action,intensity_father,intensity_son)

        # UPDATES:
        #=========
        # stochastic election of incoming active cell:
        next_activated = random_pull_dict(elligibles)
        #next_activated = max(elligibles.iteritems(), key=operator.itemgetter(1))[0]
        
        # new intensities:
        for cell in new_intensities:
            if cell not in self.modifieds:
                self.intensities[cell] = new_intensities[cell]
                self.modifieds.add(cell)

        #print str(next_activated)+ " "+ str(self.intensities[next_activated])
        #if self.action:
        #    print self.counts[self.action_number[self.action],self.cell_number[self.activateds[-1]],self.cell_number[next_activated]]
        #    print max(self.counts[self.action_number[self.action],self.cell_number[self.activateds[-1]]])

        # TODO: loop on the previous percept in past to make reinforcement with delay

        # action learning:
        if self.action:
            self.learn(next_activated,tot_reward)

        # new activated cell
        if next_activated:
            self.add_activated(next_activated)
            self.add_intensity(self.intensities[next_activated])

        # DECISION:
        #==========
        if possible_actions:
            return self.decision(possible_actions)
        else:
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

        self.cor[num_act,num_cell1,num_cell2,int(I1>0)] = ETA1*self.cor[num_act][num_cell1][num_cell2][int(I1>0)] + (1-ETA1)*I2


    def decision(self, possible_actions=None):
        state = 0
        I = 0
        if self.activateds:
            state = self.cell_number[self.activateds[-1]]
            I = self.old_intensities[-1]
        # TODO exploration based on convergence/difficulty to reach a state
        values = self.Q[state,:,int(I>0)]*np.abs(I)+np.random.rand(len(self.Q[state,:,int(I>0)]))/10
        if possible_actions:
            indices = []
            for action in possible_actions:
                indices.append(self.action_number[action])
            values = values[np.array(indices)]
        choice = softmax(values)
        #choice = np.argmax(values)
        #expect = np.max(values)
        self.expected = np.max(values)
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
            new_values = self.Q[new_state,:,int(new_intensity>0)]*np.abs(new_intensity)
            reach = np.max(new_values)

            # TD learning:
            # TODO : (using EMA instead of online average could be better for world with changing rules)
            TD = ( reward + DISCOUNT*reach - self.expected )
            n = self.n[last_state][action]+1.

            
            # classic Qlearning
            self.Q[last_state,action,int(last_intensity>0)] = (n*self.Q[last_state,action,int(last_intensity>0)] + TD)/(n+1.)
            self.n[last_state][action] += 1.
            self.matter[new_state,int(new_intensity>0)] = (n*(self.matter[new_state,int(new_intensity>0)]) + TD)/(n+1.)
            
            """
            # EMA Qlearning
            self.Q[last_state,action,int(last_intensity>0)] = ETA2*self.Q[last_state,action,int(last_intensity>0)] + (1-ETA2)*TD
            self.n[last_state][action] += 1.
            self.matter[new_state,int(new_intensity>0)] = ETA2*self.matter[new_state,int(new_intensity>0)] + (1-ETA2)*TD
            """
            """
            # EMA actor-critic
            self.V[last_state,action,last_intensity>0] = (n*self.Q[last_state,action,last_intensity>0] + TD)/(n+1.)
            self.Q[last_state,action,last_intensity>0] = (n*self.Q[last_state,action,last_intensity>0] + TD)/(n+1.)
            self.n[last_state][action] += 1.
            self.matter[new_state,new_intensity>0] = (n*(self.matter[new_state,new_intensity>0]) + TD)/(n+1.)
            """
            """
            print "last "+str(self.activateds[-1])+" "+str(last_intensity)
            print "act "+ str(self.action)
            print "new "+str(new_activated)+" "+str(new_intensity)
            print "rew "+str(TD)
            print "======================"
            """

        #else:
        #    pass


# static functions:
#------------------

def diff(model1, model2):
    dist = 0
    cell_diff = {}
    for cell_id in model1.intensities & model2.intensities:
        cell_num1 = model1.cell_number[cell_id]
        cell_num2 = model2.cell_number[cell_id]
        # Lmax for matter:
        matter = max(abs(model1.matter[cell_num1]),abs(model2.matter[cell_num2]))
        # this distance function is arbitrary, could be L2, L3 etc...
        dist += matter*abs(model1.intensities[cell_id]-model2.intensities[cell_id])
        # maybe this softmax pull could have its own theta:
        cell_diff.setdefault(cell_id,np.exp(THETA2*dist))

    # max of sofmax ?
    #choice = random_pull_dict(cell_diff)
    choice = max(cell_diff.iteritems(), key=operator.itemgetter(1))[0]

    # return the distance, if large enough, the agent want to correct the misunderstanding, starting by "choice" cell
    return dist,choice


