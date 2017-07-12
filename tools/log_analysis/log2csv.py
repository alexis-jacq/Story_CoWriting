import sys
import re
import csv
import numpy as np

choices = re.compile('WallTime: (?P<time>.*)] human choices (?P<type>\w+)')
preds = re.compile('WallTime: (?P<time>.*)] human predict (?P<type>\w+)')
judges = re.compile('WallTime: (?P<time>.*)] human juge (?P<type>\w+)')

goods = 0
good_array = []
wtfs = 0
wtf_array = []
num = 0

#with open(button_inc.csv,'w') as out:
#	fieldnames = ['label', 'time', 'type', 'value']

for i in range(17):
	interface = str(i+1)+'_inc/interface-12-stdout.log'
	main_activity = str(i+1)+'_inc/main_activity-13-stdout.log'
	try:
		with open(interface) as log:
			num+=1
			goods = 0; wtfs = 0
			for line in log.readlines():
				found_choices = choices.search(line)
				found_preds = preds.search(line)
				found_judges = judges.search(line)
				if found_judges:
					if found_judges.group('type')=="good":
						goods+=1
					if found_judges.group('type')=="wtf":
						wtfs+=1
			good_array.append(goods)
			wtf_array.append(wtfs)

	except IOError:
		pass

print np.mean(wtf_array)
print np.var(wtf_array)
print np.mean(good_array)
print np.var(good_array)