import sys
import time
import numpy as np
import operator
import story_maker as sm

# coherences
anyway = ["robot", "detective", "wine", "coffee","waltz","tango","emperor","trip","trap","rob","terrorise","scotch","Wiskhy","rhum","wine", "milk"]
noway = ["poke","badmouth","36Flip","Moulinex","lumberjack","polka","robot warlock", "robot witch", "robot lumberjack", "robot prince", "robot princess", "robot fairy", "robot wizard","spoon","ghost robot","robot fisherman", "robot dragon","blackmail", "poke", "bad-bmouth"]
sf = ["Deckard","Luc","robot","spam","robot space pioneer","space pioneer", "lazer juice","light saber","lazer gun","planet","alien", "alien robot","time travelor","robot time travelor","robot emperor", "robot general", "robot scientist", "scientist", "laboratory","spacecraft","spam"]
pirate = ["jack","robot","rob","robot pirate","pirate","rhum","saber","gun","island", "village","fisherman", "fisherman","salsa","rock","warlock", "witch","manor", "castle","island"]
midage = ["waltz","Ursula","Grimhilde","robot","terrorise","robot knight","robot princess", "robot prince","knight","prince", "princess", "fairy","tea","milk", "sword", "kingdom","ghost", "ghost robot", "dragon","witch","warlock","robot warlock", "robot witch", "emperor", "queen", "castle"]
forest = ["robot","forest","monkey", "robot monkey", "fairy", "lumberjack", "robot lumberjack", "ax", "beer","witch", "warlock","robot warlock", "robot witch", "manor", "castle", "trap" ]
science = ["Neurodark","robot","scientist", "laboratory","spam"]
robot = ["C3P8","R2D3", "Metalshin2047","DB9","Moulinex", "MRCK", "36Flip","Turbomecanoid","Dolores","spam","robot","robot pirate","robot knight","robot princess", "robot prince","robot wizard", "robot ghost", "robot warlock", "robot witch", "robot monkey", "robot lumberjack","robot blood","robot time travelor","robot emperor", "robot general", "robot scientist", "robot fairy","ghost robot","robot fisherman", "robot dragon"]
army = ["robot","general","knight", "robot knight", "robot general", "spacecraft","terrorise"]
magic = ["magic stick","robot","skeleton","terrorise","robot wizard", "wizard", "fairy", "sword", "kingdom","ghost", "robot ghost", "dragon","witch","warlock", "robot warlock", "robot witch","manor", "castle"]
contextes = {"robot":robot,"sf":sf,"pirate":pirate,"midage":midage, "forest":forest, "science":science, "army":army, "magic":magic }

women = ["Bianca","Dolores","Nosicaa","Lisa","MarineLePen","Ursula","Grimhilde"]
men = ["Jack","Deckard","Luc","Daxter","Pedro","Baltor","Psychlo", "Neurodark", "Turbomecanoid"]
robots = ["C3P8","R2D3", "Metalshin2047","DB9","Moulinex", "MRCK", "36Flip","Baltor","Psychlo", "Neurodark", "Turbomecanoid"]
gender = {"woman":women, "man":men, "robot":robots}

eta = 2.

class decision_maker:

	def __init__(self, condition):

		self.names = men+women+robots
		self.condition = condition
		self.last_human_move = ""
		self.last_decision = ""
		self.last_human_predict = ""
		self.logic_names = women+men+robots

		self.h_sf_score = 0
		self.h_pirate_score = 0
		self.h_midage_score = 0
		self.h_forest_score = 0
		self.h_science_score = 0
		self.h_army_score = 0
		self.h_magic_score = 0
		self.h_robot_score = 0
		self.h_most_likely_context = ""

		self.r_sf_score = 0
		self.r_pirate_score = 0
		self.r_midage_score = 0
		self.r_forest_score = 0
		self.r_science_score = 0
		self.r_army_score = 0
		self.r_magic_score = 0
		self.r_robot_score = 0
		self.r_most_likely_context = ""

		self.less_likely_context = ""

		self.randoms = 0
		self.coherances = 4.
		self.user_coherence = 0
		self.robot_coherence = 0

	#HACK:
	def set_SC_gender(self):
		self.logic_names = robots

	def update(self):

		if self.last_decision in sf:
			self.r_sf_score += eta
		if self.last_decision in pirate:
			self.r_pirate_score += eta
		if self.last_decision in midage:
			self.r_midage_score += eta
		if self.last_decision in forest:
			self.r_forest_score += eta
		if self.last_decision in science:
			self.r_science_score += eta
		if self.last_decision in magic:
			self.r_magic_score += eta
		if self.last_decision in army:
			self.r_army_score += eta
		if self.last_decision in robot:
			self.r_robot_score += eta

	def update_tom(self):

		if self.last_human_move in sf:
			self.h_sf_score += eta
		if self.last_human_move in pirate:
			self.h_pirate_score += eta
		if self.last_human_move in midage:
			self.h_midage_score += eta
		if self.last_human_move in forest:
			self.h_forest_score += eta
		if self.last_human_move in science:
			self.h_science_score += eta
		if self.last_human_move in magic:
			self.h_magic_score += eta
		if self.last_human_move in army:
			self.h_army_score += eta
		if self.last_human_move in robot:
			self.h_robot_score += eta

		values = {"robot":self.h_robot_score+np.random.rand()/1000.,"sf":self.h_sf_score+np.random.rand()/1000., "pirate":self.h_pirate_score+np.random.rand()/1000., "midage":self.h_midage_score+np.random.rand()/1000., "science":self.h_science_score+np.random.rand()/1000., "forest":self.h_forest_score+np.random.rand()/1000., "magic":self.h_magic_score+np.random.rand()/1000., "army":self.h_army_score+np.random.rand()/1000.}
		self.h_most_likely_context = max(values.iteritems(), key=operator.itemgetter(1))[0]

		if self.last_human_predict in sf:
			self.r_sf_score += eta
		if self.last_human_predict in pirate:
			self.r_pirate_score += eta
		if self.last_human_predict in midage:
			self.r_midage_score += eta
		if self.last_human_predict in forest:
			self.r_forest_score += eta
		if self.last_human_predict in science:
			self.r_science_score += eta
		if self.last_human_predict in magic:
			self.r_magic_score += eta
		if self.last_human_predict in army:
			self.r_army_score += eta
		if self.last_human_predict in robot:
			self.r_robot_score += eta

		values = {"robot":self.r_robot_score+np.random.rand()/1000.,"sf":self.r_sf_score+np.random.rand()/1000., "pirate":self.r_pirate_score+np.random.rand()/1000., "midage":self.r_midage_score+np.random.rand()/1000., "science":self.r_science_score+np.random.rand()/1000., "forest":self.r_forest_score+np.random.rand()/1000., "magic":self.r_magic_score+np.random.rand()/1000., "army":self.r_army_score+np.random.rand()/1000.}
		self.r_most_likely_context = max(values.iteritems(), key=operator.itemgetter(1))[0]

		values = {"robot":self.r_robot_score+self.h_robot_score+np.random.rand()/1000.,"sf":self.h_sf_score+self.r_sf_score+np.random.rand()/1000., "pirate":self.h_pirate_score+self.r_pirate_score+np.random.rand()/1000., "midage":self.h_midage_score+self.r_midage_score+np.random.rand()/1000., "science":self.r_science_score+self.h_science_score+np.random.rand()/1000., "forest":self.r_forest_score+self.h_forest_score+np.random.rand()/1000., "magic":self.r_magic_score+self.h_magic_score+np.random.rand()/1000., "army":self.r_army_score+self.h_army_score+np.random.rand()/1000.}
		print values
		self.less_likely_context = min(values.iteritems(), key=operator.itemgetter(1))[0]

		print self.r_sf_score
		print self.r_most_likely_context
		print self.less_likely_context



	def choose(self, lhm, lhp, choice):

		if lhm in gender:
			self.logic_names = gender[lhm]

		if choice[0] in women+men+robots:
			logic_choice = list(set(choice).intersection(set(self.logic_names)))
			illogic_choice = list(set(choice).intersection(set(women+men+robots)))
			for x in logic_choice:
				illogic_choice.remove(x)
		else:
			logic_choice = choice
			illogic_choice = choice

		self.last_human_move = lhm
		self.last_human_predict = lhp
		self.update_tom()

		decision = ""
		if self.condition == "predictable": # BE COHERENT take in account last move only
			context_choice = list(set(contextes[self.h_most_likely_context]).intersection(set(logic_choice)))
			larger_choice = list(set(contextes[self.h_most_likely_context]+anyway).intersection(set(logic_choice)))
			if len(context_choice)>0:
				decision = np.random.choice(context_choice)
			elif len(larger_choice)>0:
				decision = np.random.choice(larger_choice)
			else:
				decision = np.random.choice(logic_choice)

		elif self.condition == "random":
			decision = np.random.choice(choice)

		else:
			# new idea : take the prediction and moove for the opposite
			if lhp in contextes[self.r_most_likely_context]+anyway: # predict coherant
				if self.randoms>self.coherances: # coherent unprobable, then, be coherent
					context_choice = list(set(contextes[self.r_most_likely_context]+anyway).intersection(logic_choice))
					if len(context_choice)>0:
						decision = np.random.choice(context_choice)
						self.coherances += 3
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
				test = np.random.rand()
				if self.randoms<self.coherances and test>0.9: # incoherent unprobable, be incoherant as predicted
					decision = lhp
					self.randoms += 2
				elif self.randoms<self.coherances: # incoherent unprobable, be incoherant as predicted
					context_choice = list(set(contextes[self.less_likely_context]+noway).intersection(illogic_choice))
					if len(context_choice)>0:
						decision = np.random.choice(context_choice)
						self.randoms += 1
					else:
						decision = np.random.choice(illogic_choice)
				else: # coherent unprobable, then, be coherent
					context_choice = list(set(contextes[self.r_most_likely_context]+anyway).intersection(logic_choice))
					if len(context_choice)>0:
						decision = np.random.choice(context_choice)
						self.coherances += 1
					else:
						decision = np.random.choice(choice)

		if decision in gender:
			self.logic_names = gender[decision]

		if lhp in contextes[self.r_most_likely_context]+anyway+contextes[self.h_most_likely_context]:
			self.user_coherence += 1

		self.last_decision = decision

		if self.last_decision in contextes[self.r_most_likely_context]+anyway+contextes[self.h_most_likely_context]:
			self.robot_coherence += 1

		self.update()
		return decision



if __name__=="__main__":

	robot = decision_maker("incoherant")
	decision = robot.choose("space pioneer","saber",sm.C_BGj_woman)
	print decision
