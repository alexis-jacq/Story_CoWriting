import sys
import time
import numpy as np
import operator
import story_maker as sm

# coherences
anyway = ["robot blood","robot", "detective", "wine", "coffee","waltz","tango","emperor","general","scientist","trip","trap","rob","terrorise","scotch","Wiskhy","rhum","wine", "milk"]
noway = ["poke","badmouth","MarineLePen","36Flip","Moulinex","lumberjack","polka","robot warlock", "robot witch", "robot lumberjack", "robot prince", "robot princess", "robot fairy", "robot wizard","spoon","ghost robot","robot fisherman", "robot dragon","blackmail", "poke", "bad-bmouth"]
sf = ["robot","spam","robot space pioneer","space pioneer", "lazer juice","light saber","lazer gun","planet","alien", "alien robot","time travelor","robot time travelor","robot emperor", "robot general", "robot scientist", "scientist", "laboratory","spacecraft","salsa","rock","spam"]
pirate = ["robot","rob","robot pirate","pirate","rhum","saber","gun","island", "village","fisherman", "fisherman","salsa","rock","warlock", "witch","manor", "castle","island"]
midage = ["robot","terrorise","robot knight","robot princess", "robot prince","knight","prince", "princess", "fairy","tea","milk", "sword", "kingdom","ghost", "dragon","witch","warlock","robot warlock", "robot witch", "emperor","manor", "laboratory", "castle","island"]
forest = ["robot","forest","monkey", "robot monkey", "fairy", "lumberjack", "robot lumberjack", "sword", "beer","robot warlock", "robot witch", "manor", "castle", "trap" ]
science = ["robot","scientist", "laboratory","spam"]
army = ["robot","general","knight", "robot knight", "robot general", "spacecraft","terrorise"]
magic = ["robot","terrorise","robot wizard", "wizard", "fairy", "sword", "kingdom","ghost", "robot ghost", "dragon","witch","warlock", "robot warlock", "robot witch","manor", "castle"]
contextes = {"sf":sf,"pirate":pirate,"midage":midage, "forest":forest, "sciences":science, "army":army, "magic":magic }

women = ["Bianca","Dolores","Nosicaa","Lisa","MarineLePen","Ursula","Grimhilde"]
men = ["Jack","Deckard","Luc","Daxter","Pedro","Baltor","Psychlo", "Neurodark", "Turbomecanoid"]
robots = ["C3P8","R2D3", "Metalshin2047","DB9","Moulinex", "MRCK", "36Flip","Baltor","Psychlo", "Neurodark", "Turbomecanoid"]
gender = {"woman":women, "man":men, "robot":robots}


class decision_maker:

	def __init__(self, condition):

		self.condition = condition
		self.last_child_move = ""
		self.last_child_predict = ""
		self.logic_names = women+men+robots

		self.ch_sf_score = 0
		self.ch_pirate_score = 0
		self.ch_midage_score = 0
		self.ch_forest_score = 0
		self.ch_science_score = 0
		self.ch_army_score = 0
		self.ch_magic_score = 0
		self.ch_most_likely_context = ""

		self.r_sf_score = 0
		self.r_pirate_score = 0
		self.r_midage_score = 0
		self.r_forest_score = 0
		self.r_science_score = 0
		self.r_army_score = 0
		self.r_magic_score = 0
		self.r_most_likely_context = ""

		self.less_likely_context = ""

		self.randoms = 0
		self.coherances = 4.

	def update(self):

		eta = 1.

		if self.last_child_move in sf:
			self.ch_sf_score += eta
		if self.last_child_move in pirate:
			self.ch_pirate_score += eta
		if self.last_child_move in midage:
			self.ch_midage_score += eta
		if self.last_child_move in forest:
			self.ch_forest_score += eta
		if self.last_child_move in science:
			self.ch_science_score += eta
		if self.last_child_move in magic:
			self.ch_magic_score += eta
		if self.last_child_move in army:
			self.ch_army_score += eta

		values = {"sf":self.ch_sf_score+np.random.rand()/1000., "pirate":self.ch_pirate_score+np.random.rand()/1000., "midage":self.ch_midage_score+np.random.rand()/1000., "science":self.ch_science_score+np.random.rand()/1000., "forest":self.ch_forest_score+np.random.rand()/1000., "magic":self.ch_magic_score+np.random.rand()/1000., "army":self.ch_army_score+np.random.rand()/1000.}
		self.ch_most_likely_context = max(values.iteritems(), key=operator.itemgetter(1))[0]

		if self.last_child_predict in sf:
			self.r_sf_score += eta
		if self.last_child_predict in pirate:
			self.r_pirate_score += eta
		if self.last_child_predict in midage:
			self.r_midage_score += eta
		if self.last_child_predict in forest:
			self.r_forest_score += eta
		if self.last_child_predict in science:
			self.r_science_score += eta
		if self.last_child_predict in magic:
			self.r_magic_score += eta
		if self.last_child_predict in army:
			self.r_army_score += eta

		values = {"sf":self.r_sf_score+np.random.rand()/1000., "pirate":self.r_pirate_score+np.random.rand()/1000., "midage":self.r_midage_score+np.random.rand()/1000., "science":self.r_science_score+np.random.rand()/1000., "forest":self.r_forest_score+np.random.rand()/1000., "magic":self.r_magic_score+np.random.rand()/1000., "army":self.r_army_score+np.random.rand()/1000.}
		self.r_most_likely_context = max(values.iteritems(), key=operator.itemgetter(1))[0]

		values = {"sf":self.ch_sf_score+self.r_sf_score+np.random.rand()/1000., "pirate":self.ch_pirate_score+self.r_pirate_score+np.random.rand()/1000., "midage":self.ch_midage_score+self.r_midage_score+np.random.rand()/1000., "science":self.r_science_score+self.ch_science_score+np.random.rand()/1000., "forest":self.r_forest_score+self.ch_forest_score+np.random.rand()/1000., "magic":self.r_magic_score+self.ch_magic_score+np.random.rand()/1000., "army":self.r_army_score+self.ch_army_score+np.random.rand()/1000.}
		print values
		self.less_likely_context = min(values.iteritems(), key=operator.itemgetter(1))[0]

		print self.r_sf_score
		print self.r_most_likely_context
		print self.less_likely_context



	def choose(self, lcm, lcp, choice):

		if lcm in gender:
			self.logic_names = gender[lcm]

		if choice[0] in women+men+robots:
			logic_choice = list(set(choice).intersection(set(self.logic_names)))
			illogic_choice = list(set(choice).intersection(set(women+men+robots)))
			for x in logic_choice:
				illogic_choice.remove(x)
		else:
			logic_choice = choice
			illogic_choice = choice

		self.last_child_move = lcm
		self.last_child_predict = lcp
		self.update()

		decision = ""
		if self.condition == "predictable": # BE COHERENT take in account last move only
			context_choice = list(set(contextes[self.ch_most_likely_context]).intersection(set(logic_choice)))
			larger_choice = list(set(contextes[self.ch_most_likely_context]+anyway).intersection(set(logic_choice)))
			if len(context_choice)>0:
				decision = np.random.choice(context_choice)
			elif len(larger_choice)>0:
				decision = np.random.choice(larger_choice)
			else:
				decision = np.random.choice(logic_choice)

		else:
			# new idea : take the prediction and moove for the opposite
			if lcp in contextes[self.ch_most_likely_context]+anyway: # predict coherant
				if self.randoms>self.coherances: # coherent unprobable, then, be coherent
					context_choice = list(set(contextes[self.ch_most_likely_context]+anyway).intersection(logic_choice))
					if len(context_choice)>0:
						decision = np.random.choice(context_choice)
						self.coherances += 1
					else:
						decision = np.random.choice(choice)
				else: # coherent too probable, stay incoherent
					context_choice = list(set(contextes[self.less_likely_context]+noway).intersection(illogic_choice))
					if len(context_choice)>0:
						decision = np.random.choice(context_choice)
						self.randoms += 1
					else:
						decision = np.random.choice(illogic_choice)
			else: # predict incoherant
				if self.randoms<self.coherances: # incoherent unprobable, be incoherant as predicted
					decision = lcp
					self.randoms += 1
				else: # coherent unprobable, then, be coherent
					context_choice = list(set(contextes[self.ch_most_likely_context]+anyway).intersection(logic_choice))
					if len(context_choice)>0:
						decision = np.random.choice(context_choice)
						self.coherances += 1
					else:
						decision = np.random.choice(choice)

		if decision in gender:
			self.logic_names = gender[decision]
		return decision



if __name__=="__main__":

	robot = decision_maker("incoherant")
	decision = robot.choose("space pioneer","saber",sm.C_BGj_woman)
	print decision
