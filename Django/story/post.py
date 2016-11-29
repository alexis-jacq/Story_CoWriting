
from .models import *
from .serializers import *
from rest_framework.response import Response


class GetPostData():
	"""docstring for ClassName"""
		
	def object_decoder(self,obj):
		for obj_item in obj:
			#print (obj[obj_item])
			#print("----------")
			if obj_item in ['C_MC']: # main Characters name
				#print (obj[obj_item])
				
				for nameCh in range(len(obj[obj_item])):
					try:
						Ch = Characters.objects.get(namech=obj[obj_item][nameCh])
					except Characters.DoesNotExist:
						Ro = Role.objects.get(namero='Main')
						Ch = Characters(namech= obj[obj_item][nameCh], idro = Ro)
						Ch.save()

			elif obj_item in ['C_MCg']:# main Characters gender
				#print (obj[obj_item])
				for nameGe in range(len(obj[obj_item])):
					try:
						Ge = Gender.objects.get(namege=obj[obj_item][nameGe])
					except Gender.DoesNotExist:
						Ge = Gender(namege= obj[obj_item][nameGe])
						Ge.save()

			elif obj_item in ['C_MCj_man']: # main Characters job if man
				#print (obj[obj_item])
				for nameJo in range(len(obj[obj_item])):
					try:
						Jo = Job.objects.get(namejo=obj[obj_item][nameJo])
					except Job.DoesNotExist:
						Ge = Gender.objects.get(namege='man')
						Jo = Job(namejo= obj[obj_item][nameJo], idge=Ge)
						Jo.save()

			elif obj_item in ['C_MCj_woman']:# main Characters job if woman
				#print (obj[obj_item])
				for nameJo in range(len(obj[obj_item])):
					try:
						Jo = Job.objects.get(namejo=obj[obj_item][nameJo])
					except Job.DoesNotExist:
						Ge = Gender.objects.get(namege='woman')
						Jo = Job(namejo= obj[obj_item][nameJo], idge=Ge)
						Jo.save()

			elif obj_item in ['C_MCj_robot']:# main Characters job if robto
				#print (obj[obj_item])
				for nameJo in range(len(obj[obj_item])):
					try:
						Jo = Job.objects.get(namejo=obj[obj_item][nameJo])
					except Job.DoesNotExist:
						Ge = Gender.objects.get(namege='robot')
						Jo = Job(namejo= obj[obj_item][nameJo], idge=Ge)						
						Jo.save()

			elif obj_item in ['C_MCd']:# main Characters favourite Drinks
				#print (obj[obj_item])
				for nameDr in range(len(obj[obj_item])):
					try:
						Dr = Drinks.objects.get(namedr=obj[obj_item][nameDr])
					except Drinks.DoesNotExist:
						Co = Containers.objects.get(nameco='glass')
						Dr = Drinks(namedr= obj[obj_item][nameDr], idco=Co)
						Dr.save()

			elif obj_item in ['C_MCw']:# main Characters weapon
				#print (obj[obj_item])
				for nameWe in range(len(obj[obj_item])):
					try:
						We = Weapons.objects.get(namewe=obj[obj_item][nameWe])
					except Weapons.DoesNotExist:
						We = Weapons(namewe= obj[obj_item][nameWe])
						We.save()

			elif obj_item in ['C_P']:# main Characters scenario and story scenario
				#print (obj[obj_item])
				for namePl in range(len(obj[obj_item])):
					try:
						Pl = Place.objects.get(namepl=obj[obj_item][namePl])
					except Place.DoesNotExist:
						Pl = Place(namepl= obj[obj_item][namePl])
						Pl.save()
				for nameSc in range(len(obj[obj_item])):
					try:
						Sc = Scenarios.objects.get(namesc=obj[obj_item][nameSc])
					except Scenarios.DoesNotExist:
						Sc = Scenarios(namesc= obj[obj_item][nameSc])
						Sc.save()

			elif obj_item in ['C_SC']:# second Characters name
				#print (obj[obj_item])
				for nameCh in range(len(obj[obj_item])):
					try:
						Ch = Characters.objects.get(namech=obj[obj_item][nameCh])
					except Characters.DoesNotExist:
						Ro = Role.objects.get(namero='Secondary')
						Ch = Characters(namech= obj[obj_item][nameCh], idro = Ro)
						Ch.save()

			elif obj_item in ['C_SCg']:# second Characters gender
				#print (obj[obj_item])
				for nameGe in range(len(obj[obj_item])):
					try:
						Ge = Gender.objects.get(namege=obj[obj_item][nameGe])
					except Gender.DoesNotExist:
						Ge = Gender(namege= obj[obj_item][nameGe])
						Ge.save()

			elif obj_item in ['C_SCs_human']:# second Characters Species human
				#print (obj[obj_item])
				for nameSp in range(len(obj[obj_item])):
					try:
						Sp = Species.objects.get(namesp=obj[obj_item][nameSp])
					except Species.DoesNotExist:
						Sp = Species(namesp= obj[obj_item][nameSp])
						Sp.save()

			elif obj_item in ['C_SCs_robot']: # second Characters Species robot
				#print (obj[obj_item])
				for nameSp in range(len(obj[obj_item])):
					try:
						Sp = Species.objects.get(namesp=obj[obj_item][nameSp])
					except Species.DoesNotExist:
						Sp = Species(namesp= obj[obj_item][nameSp])
						Sp.save()

			elif obj_item in ['C_SC_dance']:# second Characters dance
				#print (obj[obj_item])
				for nameDa in range(len(obj[obj_item])):
					try:
						Da = Dance.objects.get(nameda=obj[obj_item][nameDa])
					except Dance.DoesNotExist:
						Da = Dance(nameda= obj[obj_item][nameDa])
						Da.save()

			elif obj_item in ['C_BG']:# bad guy name
				#print (obj[obj_item])
				for nameCh in range(len(obj[obj_item])):
					try:
						Ch = Characters.objects.get(namech=obj[obj_item][nameCh])
					except Characters.DoesNotExist:
						Ro = Role.objects.get(namero='Villain')
						Ch = Characters(namech= obj[obj_item][nameCh], idro = Ro)
						Ch.save()

			elif obj_item in ['C_BGg']:# bad guy gender
				#print (obj[obj_item])
				for nameGe in range(len(obj[obj_item])):
					try:
						Ge = Gender.objects.get(namege=obj[obj_item][nameGe])
					except Gender.DoesNotExist:
						Ge = Gender(namege= obj[obj_item][nameGe])
						Ge.save()

			elif obj_item in ['C_BGj_man']:# bad guy job man
				#print (obj[obj_item])
				for nameJo in range(len(obj[obj_item])):
					try:
						Jo = Job.objects.get(namejo=obj[obj_item][nameJo])
					except Job.DoesNotExist:
						Ge = Gender.objects.get(namege='man')
						#print Ge
						Jo = Job(namejo= obj[obj_item][nameJo], idge=Ge)
						Jo.save()

			elif obj_item in ['C_BGj_woman']:# bad guy joba woman
				#print (obj[obj_item])
				for nameJo in range(len(obj[obj_item])):
					try:
						Jo = Job.objects.get(namejo=obj[obj_item][nameJo])
					except Job.DoesNotExist:
						Ge = Gender.objects.get(namege='woman')
						#print Ge
						Jo = Job(namejo= obj[obj_item][nameJo], idge=Ge)
						Jo.save()

			elif obj_item in ['C_BGj_robot']:# bad guy job robot
				#print (obj[obj_item])
				for nameJo in range(len(obj[obj_item])):
					try:
						Jo = Job.objects.get(namejo=obj[obj_item][nameJo])
					except Job.DoesNotExist:
						Ge = Gender.objects.get(namege='robot')
						Jo = Job(namejo= obj[obj_item][nameJo], idge=Ge)
						Jo.save()

			elif obj_item in ['C_Ba']:# bad guy ticks
				#print (obj[obj_item])
				for nameTi in range(len(obj[obj_item])):
					try:
						Ti = Ticks.objects.get(nameti=obj[obj_item][nameTi])
					except Ticks.DoesNotExist:
						Ti = Ticks(nameti= obj[obj_item][nameTi])
						Ti.save()
			elif obj_item in ['C_BGp']: # bad guy place
				#print (obj[obj_item])
				for namePl in range(len(obj[obj_item])):
					#print obj[obj_item][namePl]
					try:
						Pl = Place.objects.get(namepl=obj[obj_item][namePl])
						#print Pl
					except Place.DoesNotExist:
						Pl = Place(namepl= obj[obj_item][namePl])
						Pl.save()
			elif obj_item in ['C_BGd']:# bad guy favourite Drinks
				#print (obj[obj_item])
				for nameDr in range(len(obj[obj_item])):
					try:
						Dr = Drinks.objects.get(namedr=obj[obj_item][nameDr])
					except Drinks.DoesNotExist:
						Co = Containers.objects.get(nameco='glass')
						Dr = Drinks(namedr= obj[obj_item][nameDr], idco=Co)
						Dr.save()