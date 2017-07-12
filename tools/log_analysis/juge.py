import pandas as pd
import seaborn as sns
import numpy as np
import os

from scipy.stats import ttest_ind,ttest_rel

p = "LOG_P_CSV"
s = "LOG_S_CSV"
r = "LOG_R_CSV"

file = "interface-12-stdout.log"

counters_p = {"good":[],"wtf":[],"lol":[],"bad":[],"all":[],"positive":[],"negative":[]}
counters_s = {"good":[],"wtf":[],"lol":[],"bad":[],"all":[],"positive":[],"negative":[]}
counters_r = {"good":[],"wtf":[],"lol":[],"bad":[],"all":[],"positive":[],"negative":[]}

def counts(condition, counter):
    for i in range(60):
        try:
            name = condition + '/' + str(i) + '/' + file
            table = pd.read_csv(name, dtype=str, names=['msg','_','time','subject','key','value'])
        except IOError:
            continue

        for target in counter:
            if target=="all":
                counter[target].append(sum(table['key']=="juge"))
            elif target=="positive":
                counter[target].append(sum(table['value']=="good") +sum(table['value']=="lol"))
            elif target=="negative":
                counter[target].append(sum(table['value']=="bad") +sum(table['value']=="wtf"))
            else:
                counter[target].append(sum(table['value']==target))

    return counter

counters_p = counts(p, counters_p)
counters_r = counts(r, counters_r)
counters_s = counts(s, counters_s)

key = "all"
table = np.array([counters_p[key],counters_r[key],counters_s[key]])
print(table)
print(np.mean(counters_p[key]))
print(np.mean(counters_s[key]))
print(np.sqrt(np.var(counters_p[key])))
print(np.sqrt(np.var(counters_s[key])))
print(ttest_ind(counters_p[key],counters_s[key]))

ax = sns.barplot(data=table); sns.plt.show()
