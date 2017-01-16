
# choices

C_MC = ["Jack","Bianca","C3P8","Dolores","Deckard","Luc","Nosicaa","Hal","R2D3", "Metalshin2047"]
C_MCg = ["man", "woman", "robot"]
C_MCj_man = ["pirate", "detective","knight","space pioneer", "lumberjack", "prince", "wizard"]
C_MCj_woman = ["pirate", "detective","knight","space pioneer", "lumberjack", "princess", "fairy"]
C_MCj_robot = ["robot pirate", "robot detective","robot knight","robot space pioneer", "robot lumberjack", "robot prince", "robot princess", "robot fairy", "robot wizard"]

C_MCd = ["tea", "rhum", "lazer juice", "wine", "coffee", "beer", "milk"]

C_MCw = ["light saber", "saber", "sword", "lazer gun", "gun","spoon"]

C_P = ["planet", "forest", "kingdom", "island", "village"]

C_SC = ["Daxter","Pedro","Lisa","DB9","Moulinex", "MRCK", "36Flip"]
C_SCg = ["man", "woman","robot"]
C_SCs_human= ["ghost", "alien", "monkey", "fisherman", "robot"]
C_SCs_robot= ["ghost robot", "alien robot", "robot monkey", "fisherman robot"]

C_SC_dance = ["waltz","tango","polka","salsa","rock"]
#C_Pp = add_s(C_SCs)
C_Pp = C_SCs_human

C_BG = ["Baltor","MarineLePen", "Psychlo", "Neurodark", "Turbomecanoid"]
C_BGg = ["man", "woman","robot"]
C_BGj_man = ["time travelor", "scientist", "warlock", "emperor", "general"]
C_BGj_woman = ["time travelor", "scientist", "emperor", "general", "witch"]
C_BGj_robot = ["robot time travelor", "robot scientist", "robot warlock", "robot emperor", "robot general", "robot witch"]

C_BGd = C_Pp
C_Ba = ["trip", "poke", "badmouth", "trap", "rob", "blackmail", "terrorise", "spam"]

C_BGp = ["manor", "spacecraft", "laboratory", "castle"]
C_BGdrink = ["scotch","whisky","rhum","wine", "milk","blood", "robot blood"]





class story:
	def __init__(self):
		# Default values

		self.MC = "Dolores" # main char. name (child)
		self.MCg = "robot" # main char gender (child)
		self.MCj = "space pioneer" # main char. job (robot)
		self.MCd = "tea" # main char. favorite drink (robot)
		self.MCw = "light saber" # main ch weapon (child)

		self.C_MCj = C_MCj_robot
		self.C_SCs = C_SCs_robot
		self.C_BGj = C_BGj_robot

		self.P = "kingdom" # story place (child)
		self.Pp = "alien" # place brave people (robot)

		self.SC = "DB9" # second character name (robot)
		self.SCg = "robot" # second character species (child)
		self.SCs = "alien robot"
		self.SC_dance = "polka" # (robot)

		self.BG = "Neurodark" # Bad guy (robot)
		self.BGg = "robot"
		self.BGj = "robot emperor" # bad guy job (child)
		self.BGd = "fisherman" # bad guy dogs (robot)
		self.Ba = "badmouth" # nefast action perfomed by bad guy's dogs (child)
		self.BGp = "manor" # bad guy place (child)
		self.BGdrink = "rum" # bad guy drink (robot)

		# dependencies

		self.C_MC_vehicule = {"pirate":"boat", "detective":"car","knight":"horse","space pioneer":"spaceship", "lumberjack":"truck","prince":"horse", "princess":"unicorn", "witch":"broom","wizard":"broom", "fairy":"dragonfly"}
		self.C_MC_vehicule_place = {"pirate":"port", "detective":"parking","knight":"stable","prince":"stable","space pioneer":"space port", "lumberjack":"warhouse", "princess":"rainbow", "witch":"cupboard","wizard":"cupboard", "fairy":"stream"}
		self.C_MC_transport = {"pirate":"sailed", "detective":"drove","knight":"rode","prince":"rode","space pioneer":"sailed", "lumberjack":"drove", "princess":"rode", "witch":"flyed","wizard":"flyed", "fairy":"rode"}
		self.C_MCd_glass = {"tea":"cup", "rhum":"glass", "lazer juice":"glass", "wine":"glass", "coffee":"cup", "beer":"pint", "milk":"glass"}
		self.C_BG_title = {"time travelor": "Professor", "scientist":"Professor", "warlock":"Master", "emperor":"sir", "general":"general", "witch":""}
		self.C_Ba_part = {"trip":" up", "poke":"", "badmouth":"", "prod":"", "rob":"", "blackmail":"", "terrorise":"","trap":"", "terrorise":"","spam":""}
		self.C_Ba_target = {"trip":" walker", "poke":"walker", "badmouth":self.Pp, "rob":"walker", "blackmail":self.Pp, "trap":"walker","terrorise":"walker","spam":self.Pp}
		self.C_Ba_result = {"trip":"trip", "poke":"poke", "badmouth":"critic", "rob":"rob", "blackmail":"blackmail", "trap":"trap", "terrorise":"terrorise","spam":"spam"}
		self.C_BG_tool = {"time travelor":"computer", "scientist":"computer", "warlock":"spell", "emperor":"computer", "general":"computer","witch":"spell"}

		# Dependent values

		self.MC_vehicule = self.C_MC_vehicule[self.MCj.replace("robot ","")]
		self.MC_vehicule_place = self.C_MC_vehicule_place[self.MCj.replace("robot ","")]
		self.MC_transport = self.C_MC_transport[self.MCj.replace("robot ","")]
		self.MCd_glass = self.C_MCd_glass[self.MCd]
		self.BG_title = self.C_BG_title[self.BGj.replace("robot ","")]
		self.BGt = self.C_BG_tool[self.BGj.replace("robot ","")]
		self.Ba_part = self.C_Ba_part[self.Ba]
		self.Ba_result = self.C_Ba_result[self.Ba]
		self.Ba_target = self.C_Ba_target[self.Ba]

		self.MC_ppos = "its"
		self.MC_pper_s = "it"
		self.MC_pper_o = "it"
		self.SC_ppos = "its"
		self.SC_pper_s = "it"
		self.SC_pper_o = "it"
		self.BG_ppos = "its"
		self.BG_pper_s = "it"
		self.BG_pper_o = "it"

		self.phrase1 = "."
		if self.Ba in ["trip","trap"]:
			self.phrase1 = " as soon as they were trying to walk:. ''No more "+self.Pp+" dare to have a walk outside since every ten meters, an "+self.BGd+" leg arises for a "+self.Ba_result+""+self.Ba_part+" and then the walking "+self.Pp+" fall down, nose on the floor''."
		if self.Ba in ["poke"]:
			self.phrase1 = " as soon as they were going outside:. ''No more "+self.Pp+" dare to have a walk outside since every ten meters, an "+self.BGd+" hand arises for a "+self.Ba_result+""+self.Ba_part+"."
		if self.Ba in ["robe"]:
			self.phrase1 = " as soon as they were not paying attention:. ''No more "+self.Pp+" dare to have a walk outside since every ten meters, an "+self.BGd+" hand arises and "+self.Ba+" everything they have in their pokets."
		if self.Ba in ["badmouth"]:
			self.phrase1 = ":. ''No more "+self.Pp+" dare to have a walk outside since every ten meters, a team of "+self.BGd+" arises and launches hundreds of "+self.Ba_result+"s."
		if self.Ba in ["blackmail"]:
			self.phrase1 = ":. ''No more "+self.Pp+" dare to have a walk outside since every ten meters, a team of "+self.BGd+" arises and finds a way to "+self.Ba+" the poor passing "+self.Pp+"."


	def update(self):

		self.C_Ba_target = {"trip":" walker", "poke":"walker", "badmouth":self.Pp, "rob":"walker", "blackmail":self.Pp, "trap":"walker","terrorise":"walker","spam":self.Pp}

		self.MC_vehicule = self.C_MC_vehicule[self.MCj.replace("robot ","")]
		self.MC_vehicule_place = self.C_MC_vehicule_place[self.MCj.replace("robot ","")]
		self.MC_transport = self.C_MC_transport[self.MCj.replace("robot ","")]
		self.MCd_glass = self.C_MCd_glass[self.MCd]
		self.BG_title = self.C_BG_title[self.BGj.replace("robot ","")]
		self.BGt = self.C_BG_tool[self.BGj.replace("robot ","")]
		self.Ba_part = self.C_Ba_part[self.Ba]
		self.Ba_result = self.C_Ba_result[self.Ba]
		self.Ba_target = self.C_Ba_target[self.Ba]

		if self.MCg=="man":
			self.MC_ppos = "his"
			self.MC_pper_s = "he"
			self.MC_pper_o = "him"
			self.C_MCj = C_MCj_man
		if self.MCg=="woman":
			self.MC_ppos = "her"
			self.MC_pper_s = "she"
			self.MC_pper_o = "her"
			self.C_MCj = C_MCj_woman

		if self.SCg=="man":
			self.SC_ppos = "his"
			self.SC_pper_s = "he"
			self.SC_pper_o = "him"
			self.C_SCs = C_SCs_human
		if self.SCg=="woman":
			self.SC_ppos = "her"
			self.SC_pper_s = "she"
			self.SC_pper_o = "her"
			self.C_SCs = C_SCs_human

		if self.BGg=="man":
			self.BG_ppos = "his"
			self.BG_pper_s = "he"
			self.BG_pper_o = "him"
			self.C_BGj = C_BGj_man
		if self.BGg=="woman":
			self.BG_ppos = "her"
			self.BG_pper_s = "she"
			self.BG_pper_o = "her"
			self.C_BGj = C_BGj_woman

		self.phrase1 = "."
		if self.Ba in ["trip","trap"]:
			self.phrase1 = " as soon as they were trying to walk:. ''No more "+self.Pp+" dare to have a walk outside since every ten meters, an "+self.BGd+" leg arises for a "+self.Ba_result+""+self.Ba_part+" and then the walking "+self.Pp+" fall down, nose on the floor''."
		if self.Ba in ["poke"]:
			self.phrase1 = " as soon as they were going outside:. ''No more "+self.Pp+" dare to have a walk outside since every ten meters, an "+self.BGd+" hand arises for a "+self.Ba_result+""+self.Ba_part+"."
		if self.Ba in ["robe"]:
			self.phrase1 = " as soon as they were not paying attention:. ''No more "+self.Pp+" dare to have a walk outside since every ten meters, an "+self.BGd+" hand arises and "+self.Ba+" everything they have in their pokets."
		if self.Ba in ["badmouth"]:
			self.phrase1 = ":. ''No more "+self.Pp+" dare to have a walk outside since every ten meters, a team of "+self.BGd+"s arises and launches hundreds of "+self.Ba_result+"s."
		if self.Ba in ["blackmail"]:
			self.phrase1 = ":. ''No more "+self.Pp+" dare to have a walk outside since every ten meters, a team of "+self.BGd+" arises and finds a way to "+self.Ba+" the poor passing "+self.Pp+"."


	def generate(self):
		self.update()
		return "Once upon a time, in a "+self.P+" far away populated by "+self.Pp+"s"+", was living a wild "+self.MCj+" named "+self.MC+". "+self.MC+", subtle amateur of "+self.MCd+", was tall and thin and mastered the fight to the "+self.MCw+". But in these peacful times, no works were requiring a "+self.MCj+"'s skills. while "+self.MC_pper_s+" was dreaming of adventur, the brave "+self.Pp+"s"+" of the "+self.P+" were borring "+self.MC_pper_o+". That's when, a morning, a "+self.SCs+" named "+self.SC+" cam to meet "+self.MC_pper_o+". "+self.SC+" was not so tall, more or less creepy but, however, elegant. "+self.SC_pper_s+" seemed to be affraid but "+self.SC_pper_s+" was successfully keeping "+self.SC_ppos+" calm. "+self.MC+" suggested a "+self.MCd_glass+" of "+self.MCd+". "+self.SC+" explained that a crazy "+self.BGj+", the maleficent "+self.BG_title+" "+self.BG+", was sending hordes of "+self.BGd+"s"+" to "+self.Ba+" everyone"+self.Ba_part+self.phrase1+" ''That's enough! said "+self.MC+". I'm on the way to stop this ignoble "+self.BG_title+" "+self.BG+" and to make "+self.BG_pper_o+" pay "+self.BG_ppos+" offences! '' Thereby "+self.MC+" started "+self.MC_ppos+" quest of the terrible "+self.BG_title+" "+self.BG+"'s "+self.BGp+". After walking up to the "+self.MC_vehicule_place+" and having suffered of twenty "+self.BGd+" "+self.Ba_result+"s"+self.Ba_part+", "+self.MC_pper_s+" reached "+self.MC_ppos+" "+self.MC_vehicule+". Then "+self.MC_pper_s+" "+self.MC_transport+" seven days and finally arrived at the entrance of the dark house. The door was wide open... "+self.MC+" entered and, as a surprise, "+self.MC_pper_s+" found the "+self.BG_title+" in a sad look, wallowing in "+self.BG_ppos+" misery in front of a bottle of "+self.BGdrink+". After serving a glass for himself, "+self.MC+" asked what misfortune led the "+self.BG_title+" into such a state. Ah! exclamed the "+self.BG_title+", Those "+self.BGd+"... I did not send them. That was my "+self.BGt+". I just aimed to provide it with a bit more autonomy but it came to far and I lost control. Now it's sending hordes of "+self.BGd+"s"+" everywhere in order to "+self.Ba+""+self.Ba_part+" any single "+self.Ba_target+" of this "+self.P+". ''How to stop this "+self.BGt+"? '' asked "+self.MC+". ''There are no ways '', answered the "+self.BGj+". ''I tried everithing ''. But, while "+self.MC+" and the "+self.BGj+" were talking, "+self.SC+" was approaching the unchained "+self.BGt+". As if "+self.SC_pper_s+" was bewitched, "+self.SC_pper_s+" was slowly walking without fear, staring at the "+self.BGt+". Out of the blue, a "+self.SC_dance+" melody filled the room. "+self.SC+" gave a hand and asked for a dance. Shyly, the "+self.BGt+" accepted and they danced the most beautifull "+self.SC_dance+" ever danced by a "+self.BGt+" and a "+self.SCs+". The "+self.BGt+" stopped its eerie program and "+self.BGd+"s"+" returned from where they came. On the "+self.P+", everyone started dancing. Then "+self.MC+" went back home, served to "+self.MC_pper_o+"self a "+self.MCd_glass+" of "+self.MCd+" which "+self.MC_pper_s+" drunk down. ''Damn it! '' "+self.MC_pper_s+" said, subjugated. ''What a story! '' "
