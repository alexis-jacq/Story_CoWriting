import pandas as pd
import seaborn as sns
import numpy as np
import os

from scipy.stats import ttest_ind,ttest_rel

p = "LOG_P_CSV"
s = "LOG_S_CSV"
r = "LOG_R_CSV"

file_subject = "interface-12-stdout.log"
file_robot = "main_activity-13-stdout.log"

counters_p = []
counters_s = []
counters_r = []

def counts(condition, counter):
    for i in range(60):
        try:
            name_sub = condition + '/' + str(i) + '/' + file_subject
            name_rob = condition + '/' + str(i) + '/' + file_robot
            table_sub = pd.read_csv(name_sub, dtype=str, names=['msg','_','time','subject','key','value'])
            table_rob = pd.read_csv(name_rob, dtype=str, names=['msg','_','time','subject','value','_'])
        except IOError:
            continue

        table_sub = table_sub[table_sub["key"]=="predict"]
        table_rob = table_rob[table_rob["subject"]=="robot_choice"]
        try:
            counter.append(np.sum(np.equal(table_sub["value"].values,table_rob["value"].values[::2])))
        except ValueError:
            counter.append(np.sum(np.equal(table_sub["value"].values,table_rob["value"].values[::2][:-1])))

    return counter

counters_p = counts(p, counters_p)
counters_r = counts(r, counters_r)
counters_s = counts(s, counters_s)

print(ttest_ind(counters_p,counters_s))
print(ttest_ind(counters_p,counters_r))
print(ttest_ind(counters_s,counters_r))

table = np.array([counters_p,counters_r,counters_s])
ax = sns.barplot(data=table); sns.plt.show()
