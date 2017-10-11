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
army = ["robot","general","knight", "robot knight", "robot general", "spacecraft","terrorise"]
robots = ["C3P8","R2D3", "Metalshin2047","DB9","Moulinex", "MRCK", "36Flip","Turbomecanoid","Dolores","spam","robot","robot pirate","robot knight","robot princess", "robot prince","robot wizard", "robot ghost", "robot warlock", "robot witch", "robot monkey", "robot lumberjack","robot blood","robot time travelor","robot emperor", "robot general", "robot scientist", "robot fairy","ghost robot","robot fisherman", "robot dragon"]
women_names = ["woman","Bianca","Dolores","Nosicaa","Lisa","MarineLePen","Ursula","Grimhilde"]
men_names = ["man","Jack","Deckard","Luc","Daxter","Pedro","Baltor","Psychlo", "Neurodark", "Turbomecanoid"]
robots_names = ["robot","C3P8","R2D3", "Metalshin2047","DB9","Moulinex", "MRCK", "36Flip","Turbomecanoid","Dolores"]
magic = ["magic stick","robot","skeleton","terrorise","robot wizard", "wizard", "fairy", "sword", "kingdom","ghost", "robot ghost", "dragon","witch","warlock", "robot warlock", "robot witch","manor", "castle"]

CONTEXTS = {"women_names":women_names,\
				"men_names":men_names,\
				"robots_names":robots_names,\
				"robots":robots,\
				"magic":magic,\
				"army":army,\
				"science":science,\
				"forest":forest,\
				"midage":midage,\
				"pirate":pirate,\
				"sf":sf}

eta = 2.

class model:

	def __init__(self):

		self.last_human_decision = ""
		self.last_robot_decision = ""
		self.last_human_prediction = ""

		self.contexts_scores = {"women_names":np.random.randn()*1e-5,\
						"men_names":np.random.randn()*1e-5,\
						"robots_names":np.random.randn()*1e-5,\
						"robots":np.random.randn()*1e-5,\
						"magic":np.random.randn()*1e-5,\
						"army":np.random.randn()*1e-5,\
						"science":np.random.randn()*1e-5,\
						"forest":np.random.randn()*1e-5,\
						"midage":np.random.randn()*1e-5,\
						"pirate":np.random.randn()*1e-5,\
						"sf":np.random.randn()*1e-5}

		self.most_likely_context = "sf"
		self.less_likely_context = "noway"

	def update(self, decision, weight):

		for context in CONTEXTS:
			if decision in CONTEXTS[context]:
				self.contexts_scores[context] += weight + np.random.randn()*1e-5

		# special condition for names:
		if decision in women_names:
			self.women_names_score = np.Inf
			self.men_names_score = 0.
			self.robots_names_score = -1.
		if decision in men_names:
			self.women_names_score = 0.
			self.men_names_score = np.Inf
			self.robots_names_score = -1.
		if decision in robots_names:
			self.women_names_score = np.random.randn()*1e-5
			self.men_names_score = np.random.randn()*1e-5
			self.robots_names_score = np.Inf

		self.most_likely_context = max(self.contexts_scores.iteritems(), key=operator.itemgetter(1))[0]
		self.less_likely_context = min(self.contexts_scores.iteritems(), key=operator.itemgetter(1))[0]

class decision_maker:

	def __init__(self, condition):
		self.condition = condition # surprising / predicable / random
		self.model_h = model()
		self.model_r = model()
		self.model_p = model()
		self.sh = 1.
		self.sr = 1.
		self.sp = 1.
		self.wh = np.random.randn()*1e-5
		self.wr = np.random.randn()*1e-5
		self.wp = np.random.randn()*1e-5
		self.last_decision = ""

	def update_weights(self, last_human_prediction):
		# update si:
		self.sh += (float)(last_human_prediction in CONTEXTS[self.model_h.most_likely_context])
		self.sr += (float)(last_human_prediction in CONTEXTS[self.model_r.most_likely_context])
		self.sp += (float)(last_human_prediction in CONTEXTS[self.model_p.most_likely_context])
		# update wi:
		self.wh = self.sh/(self.sh + self.sr + self.sp)
		self.wr = self.sr/(self.sh + self.sr + self.sp)
		self.wp = self.sp/(self.sh + self.sr + self.sp)

	def choose(self, last_human_decision, last_human_prediction, choice):
		# update weights:
		self.update_weights(last_human_prediction)
		# update agents models:
		self.model_h.update(last_human_decision, eta)
		self.model_r.update(self.last_decision, eta)
		# update mutual model:
		self.model_p.update(last_human_decision, self.wh)
		self.model_p.update(self.last_decision, self.wr)
		self.model_p.update(last_human_prediction, self.wp)

		# make a new decision (depending on condition):
		decision = ""
		if self.condition == "predictable": # BE COHERENT take in account last move only
			likely_choice = list(set(CONTEXTS[self.model_p.most_likely_context]).intersection(set(choice)))
			larger_choice = list(set(CONTEXTS[self.model_p.most_likely_context]+anyway).intersection(set(choice)))
			if len(likely_choice)>0:
				decision = np.random.choice(likely_choice)
			elif len(larger_choice)>0:
				decision = np.random.choice(larger_choice)
			else:
				decision = np.random.choice(choice)

		elif self.condition == "random":
			decision = np.random.choice(choice)

		else:
			if last_human_prediction in CONTEXTS[self.model_p.most_likely_context]+anyway: # predict coherant
				random = np.random.rand()
				if random<0.2: # be coherent
					likely_choice = list(set(CONTEXTS[self.model_p.most_likely_context]).intersection(set(choice)))
					larger_choice = list(set(CONTEXTS[self.model_p.most_likely_context]+anyway).intersection(set(choice)))
					if len(likely_choice)>0:
						decision = np.random.choice(likely_choice)
					elif len(larger_choice)>0:
						decision = np.random.choice(larger_choice)
					else:
						decision = np.random.choice(choice)
				else: # be incoherent
					unlikely_choice = list(set(CONTEXTS[self.model_p.less_likely_context]).intersection(set(choice)))
					larger_choice = list(set(CONTEXTS[self.model_p.less_likely_context]+noway).intersection(set(choice)))
					if len(unlikely_choice)>0:
						decision = np.random.choice(unlikely_choice)
					elif len(larger_choice)>0:
						decision = np.random.choice(larger_choice)
					else:
						decision = np.random.choice(choice)

			else: # predict uncoherant
				random = np.random.rand()
				if random<0.2: # be uncoherent and choose what human predicts
					decision = last_human_prediction
				else: # be coherent
					likely_choice = list(set(CONTEXTS[self.model_p.most_likely_context]).intersection(set(choice)))
					larger_choice = list(set(CONTEXTS[self.model_p.most_likely_context]+anyway).intersection(set(choice)))
					if len(likely_choice)>0:
						decision = np.random.choice(likely_choice)
					elif len(larger_choice)>0:
						decision = np.random.choice(larger_choice)
					else:
						decision = np.random.choice(choice)

		self.last_decision = decision
		return decision



if __name__=="__main__":

	robot = decision_maker("surprising")
	decision = robot.choose("man","Luc",sm.C_MC)
	print decision
