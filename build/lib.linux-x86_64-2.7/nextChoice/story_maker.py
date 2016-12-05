
# choices

C_MC = []
C_MCg = ["man", "woman", "robot"]
C_MCj_man = ["pirate", "detective","knight","space pioneer", "lumberjack", "prince", "wizard"]
C_MCj_woman = ["pirate", "detective","knight","space pioneer", "lumberjack", "princess", "fairy"]
C_MCj_robot = ["robot pirate", "robot detective","robot knight","robot space pioneer", "robot lumberjack", "robot prince", "robot princess", "robot fairy", "robot wizard"]

C_MCd = ["tea", "rhum", "lazer juice", "wine", "coffee", "bear", "milk"]

C_MCw = ["light sabre", "sabre", "sword", "lazergun", "gun","spoon"]

C_P = ["planet", "forest", "kingdom", "island", "village"]

C_SC = []
C_SCg = ["man", "woman","robot"]
C_SCs_human= ["ghost", "alien", "monkey", "fisherman", "robot"]
C_SCs_robot= ["ghost robot", "alien robot", "robot monkey", "fisherman robot"]

C_SC_dance = ["waltz","tango","polka","salsa","rock"]
#C_Pp = add_s(C_SCs)
C_Pp = C_SCs_human

C_BG = []
C_BGg = ["man", "woman","robot"]
C_BGj_man = ["Time travelor", "scientist", "warlock", "emperor", "general"]
C_BGj_woman = ["Time travelor", "scientist", "emperor", "general", "witch"]
C_BGj_robot = ["robot Time travelor", "robot scientist", "robot warlock", "robot emperor", "robot general", "robot witch"]

C_BGd = C_Pp
C_Ba = ["trip", "poke", "bad-bmouth", "trap", "rob", "blackmail", "terrorise"]

C_BGp = ["manor", "spacecraft", "laboratory", "castle"]
C_BGd = ["scotch","Wiskhy","rhum","wine", "milk","blood", "robot blood"]



# Default values

MC = "Jack" # main char. name (child)
MCg = "man" # main char gender (child)
MCj = "pirate" # main char. job (robot)
MCd = "tea" # main char. favorite drink (robot)
MCw = "light sabre" # main ch weapon (child)

P = "Planet" # story place (child)
Pp = "ghosts" # place brave people (robot)

SC = "Baxter" # second character name (robot)
SCg = "robot" # second character species (child)
SCs = "robot monkey"
SC_dance = "waltz" # (robot)

BG = "Albatar" # Bad guy (robot)
BGg = "man"
BGj = "time travelor" # bad guy job (child)
BGd = "alien" # bad guy dogs (robot)
BGa = "bad-mouth" # nefast action perfomed by bad guy's dogs (child)
BGp = "manor" # bad guy place (child)
BGd = "rum" # bad guy drink (robot)

# dependencies

C_MC_vehicule = {"pirate":"boat", "detective":"car","knight":"horse","space pioneer":"spaceship", "lumberjack":"truck", "princess":"unicorn", "witch":"broom", "fairy":"dragonfly"}
C_MC_vehicule_place = {"pirate":"port", "detective":"parking","knight":"stable","space pioneer":"space port", "lumberjack":"warhouse", "princess":"rainbow", "witch":"cupboard", "fairy":"stream"}
C_MC_transport = {"pirate":"sailed", "detective":"drove","knight":"rode","space pioneer":"sailed", "lumberjack":"drove", "princess":"rode", "witch":"flyed", "fairy":"rode"}
C_MCd_glass = {"tea":"cup", "rhum":"glass", "lazer juice":"glass", "wine":"glass", "coffee":"cup", "beer":"pint", "milk":"glass"}
C_BG_title = {"time travelor": "Professor", "scientist":"Professor", "wizard":"Master", "emperor":"sir", "general":"general", "witch":""}
C_Ba_part = {"trip":" up", "poke":"", "bad-mouth":"", "prod":"", "rob":"", "blackmail":"", "terrorise":""}
C_Ba_target = {"trip":" walker", "poke":"walker", "bad-mouth":Pp, "prod":"walker", "rob":"walker", "blackmail":Pp, "trap":"walker"}
C_Ba_result = {"trip":"trip", "poke":"poke", "bad-mouth":"critic", "rob":"rob", "blackmail":"blackmail", "trap":"trap"}
C_BG_tool = {"time travelor":"computer", "scientist":"computer", "wizard":"spell", "emperor":"computer", "general":"computer","witch":"spell"}

# Dependent values

MC_vehicule = C_MC_vehicule[MCj]
MC_vehicule_place = C_MC_vehicule_place[MCj]
MC_transport = C_MC_transport[MCj]
MCd_glass = C_MCd_glass[MCd]
BG_title = C_BG_title[BGj]
BGt = C_BG_tool[BGj]
BGa_part = C_Ba_part[BGa]
BGa_result = C_Ba_result[BGa]
BGa_target = C_Ba_target[BGa]

phrase1 = "."
if BGa in ["trip","trap"]:
	phrase1 = " as soon as they were trying to walk: ''No more "+Pp+" dare to have a walk outside since every ten meters, an "+BGd+" leg arises for a "+BGa_result+""+BGa_part+" and then the walking "+Pp+" fall down, nose on the floor''."
if BGa in ["poke"]:
	phrase1 = " as soon as they were going outside: ''No more "+Pp+" dare to have a walk outside since every ten meters, an "+BGd+" hand arises for a "+BGa_result+""+BGa_part+"."
if BGa in ["robe"]:
	phrase1 = " as soon as they were not paying attention: ''No more "+Pp+" dare to have a walk outside since every ten meters, an "+BGd+" hand arises and "+BGa+" everything they have in their pokets."
if BGa in ["bad-mouth"]:
	phrase1 = ": ''No more "+Pp+" dare to have a walk outside since every ten meters, a team of "+BGd+" arises and launches hundreds of "+BGa_result+"s."
if BGa in ["blackmail"]:
	phrase1 = ": ''No more "+Pp+" dare to have a walk outside since every ten meters, a team of "+BGd+" arises and finds a way to "+BGa+" the poor passing "+Pp+"."


MC_ppos = "its"
MC_pper_s = "it"
MC_pper_o = "it"
if MCg=="man":
	MC_ppos = "his"
	MC_pper_s = "he"
	MC_pper_o = "him"
if MCg=="woman":
	MC_ppos = "her"
	MC_pper_s = "she"
	MC_pper_o = "her"

SC_ppos = "its"
SC_pper_s = "it"
SC_pper_o = "it"
if SCg=="man":
	SC_ppos = "his"
	SC_pper_s = "he"
	SC_pper_o = "him"
if SCg=="woman":
	SC_ppos = "her"
	MC_pper_s = "she"
	MC_pper_o = "her"

BG_ppos = "its"
BG_pper_s = "it"
BG_pper_o = "it"
if BGg=="man":
	BG_ppos = "his"
	BG_pper_s = "he"
	BG_pper_o = "him"
if BGg=="woman":
	BG_ppos = "her"
	BG_pper_s = "she"
	BG_pper_o = "her"

if __name__=="__main__":

	story = "Once upon a time, on a "+P+" far away populated by "+Pp+"s"+", was living a wild "+MCj+" named "+MC+". "+MC+", subtle amateur of "+MCd+", was tall and thin and mastered the fight to the "+MCw+". But in these peacful times, no works were requiring a "+MCj+"'s skills. while "+MC_pper_s+" was dreaming of adventur, the brave "+Pp+"s"+" of the "+P+" were borring "+MC_pper_o+". That's when, a morning, a "+SCs+" named "+SC+" cam to meet "+MC_pper_o+". "+SC+" was not so tall, more or less creepy but, however, elegant. "+SC_pper_s+" seemed to be affraid but "+SC_pper_s+" was successfully keeping "+SC_ppos+" calm. "+MC+" suggested a "+MCd_glass+" of "+MCd+". "+SC+" explained that a crazy "+BGj+", the maleficent "+BG_title+" "+BG+", was sending hordes of "+BGd+"s"+" to "+BGa+" everyone"+BGa_part+phrase1+" ''That's enough! said "+MC+". I'm on the way to stop this ignoble "+BG_title+" "+BG+" and to make "+BG_pper_o+" pay "+BG_ppos+" offences! '' Thereby "+MC+" started "+MC_ppos+" quest of the terrible "+BG_title+" "+BG+"'s "+BGp+". After walking up to the "+MC_vehicule_place+" and having suffered of twenty "+BGd+" "+BGa_result+"s"+BGa_part+", "+MC_pper_s+" reached "+MC_ppos+" "+MC_vehicule+". Then "+MC_pper_s+" "+MC_transport+" seven days and finally arrived at the entrance of the dark house. The door was wide open... "+MC+" entered and, as a surprise, "+MC_pper_s+" found the "+BG_title+" in a sad look, wallowing in "+BG_ppos+" misery in front of a bottle of "+BGd+". After serving a glass for himself, "+MC+" asked what misfortune led the "+BG_title+" into such a state. Ah! exclamed the "+BG_title+" Those "+BGd+"... I did not send them. That was my "+BGt+". I just aimed to provide it with a bit more autonomy but it came to far and I lost control. Now it's sending hordes of "+BGd+"s"+" everywhere in order to "+BGa+""+BGa_part+" any single "+BGa_target+" of this "+P+". ''How to stop this "+BGt+"? '' asked "+MC+". ''There are no ways '', answered the "+BGj+". ''I tried everithing ''. But, while "+MC+" and the "+BGj+" were talking, "+SC+" was approaching the unchained "+BGt+". As if "+SC_pper_s+" was bewitched, "+SC_pper_s+" was slowly walking without fear, staring at the "+BGt+". Out of the blue, a "+SC_dance+" melody filled the room. "+SC+" gave a hand and asked for a dance. Shyly, the "+BGt+" accepted and they danced the most beautifull "+SC_dance+" ever danced by a "+BGt+" and a "+SCs+". The "+BGt+" stopped its eerie program and "+BGd+"s"+" returned from where they came. On the "+P+", everyone started dancing. Then "+MC+" went back home, served to "+MC_pper_o+"self a "+MCd_glass+" of "+MCd+" which "+MC_pper_s+" drunk down. ''Damn it! '' "+MC_pper_s+" said, subjugated. ''What a story! '' "
	print story
