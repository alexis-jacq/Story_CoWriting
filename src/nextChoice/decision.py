import sys
import time
import numpy as np
import operator
import story_maker as sm

# coherences
anyway = ["man", "woman", "detective", "wine", "coffee","waltz","tango","polka","emperor","general","trip","trap","rob","terrorise","scotch","Wiskhy","rhum","wine", "milk"]
noway = ["lumberjack","robot warlock", "robot witch", "robot lumberjack", "robot prince", "robot princess", "robot fairy", "robot wizard","spoon","ghost robot","robot fisherman", "robot dragon","blackmail", "poke", "bad-bmouth"]
sf = ["robot detective","robot knight","robot space pioneer","robot pirate","space pioneer","robot", "lazer juice","light sabre","gun","lazergun","planet","alien", "alien robot", "robot monkey","Time travelor","robot Time travelor","robot emperor", "robot general", "robot scientist", "scientist","spacecraft","salsa","rock"]
pirate = ["pirate","rhum","sabre", "sword","gun","island", "village","ghost","monkey", "fisherman","salsa","rock","warlock", "witch","manor", "castle","island"]
midage = ["knight","prince", "wizard", "princess", "fairy","tea","milk","sabre", "sword", "forest", "kingdom", "island", "village","ghost", "monkey", "dragon", "scientist","warlock","manor", "laboratory", "castle","island"]
contextes = {"sf":sf,"pirate":pirate,"midage":midage}


class decision_maker(condition):

	self.condition = condition
	self.last_child_move = ""
	self.last_child_predict = ""

	self.ch_sf_score = 0
	self.ch_pirate_score = 0
	self.ch_midage_score = 0
	self.ch_most_likely_context = ""

	self.r_sf_score = 0
	self.r_pirate_score = 0
	self.r_midage_score = 0
	self.r_most_likely_context = ""

	self.less_likely_context = ""

	self.randoms = 0
	self.coherances = 1.

def update(lcm,lcp):

	if lcm in sf:
		self.ch_sf_score += 1
	if lcm in pirate:
		self.ch_pirate_score += 1
	if lcm in midage:
		self.ch_midage_score += 1
	values = {"sf":ch_sf_score+np.random.rand()/1000., "pirate":ch_pirate_score+np.random.rand()/1000., "midage":ch_midage_score+np.random.rand()/1000.}
	self.ch_most_likely_context = max(values.iteritems(), key=operator.itemgetter(1))[0]

	if lcp in sf:
		self.r_sf_score += 1
	if lcp in pirate:
		self.r_pirate_score += 1
	if lcp in midage:
		self.r_midage_score += 1
	values = {"sf":r_sf_score+np.random.rand()/1000., "pirate":r_pirate_score+np.random.rand()/1000., "midage":r_midage_score+np.random.rand()/1000.}
	self.r_most_likely_context = max(values.iteritems(), key=operator.itemgetter(1))[0]

	values = {"sf":ch_sf_score+r_sf_score+np.random.rand()/1000., "pirate":ch_pirate_score+r_pirate_score+np.random.rand()/1000., "midage":ch_midage_score+r_midage_score+np.random.rand()/1000.}
	self.less_likely_context = min(values.iteritems(), key=operator.itemgetter(1))[0]


def choose(self, lcm, lcp, choice):

	self.update(lcm,lcp)

	decision = ""
	if self.condition == "coherant":
		context_choice = list(set(contextes[self.r_most_likely_context]+anyway).intersection(choice))
		decision = np.random.choice(context_choice)

	else:
		if lcp in contextes[self.r_most_likely_context]+anyway: # predict coherant
			if self.randoms>self.coherances:
				context_choice = list(set(contextes[self.r_most_likely_context]+anyway).intersection(choice))
				if len(context_choice)>0:
					decision = np.random.choice(context_choice)
				else:
					decision = np.random.choice(choice)
				if decision in sf:
					self.r_sf_score += 1
				if decision in pirate:
					self.r_pirate_score += 1
				if decision in midage:
					self.r_midage_score += 1
			else:
				context_choice = list(set(contextes[self.less_likely_context]+noway).intersection(choice))
				if len(context_choice)>0:
					decision = np.random.choice(context_choice)
				else:
					decision = np.random.choice(choice)
		else: # predict incoherant
			if self.randoms>self.coherances:
				decision = lcp
			else:
				context_choice = list(set(contextes[self.r_most_likely_context]+anyway).intersection(choice))
				if len(context_choice)>0:
					decision = np.random.choice(context_choice)
				else:
					decision = np.random.choice(choice)
				if decision in sf:
					self.r_sf_score += 1
				if decision in pirate:
					self.r_pirate_score += 1
				if decision in midage:
					self.r_midage_score += 1


	return decision










