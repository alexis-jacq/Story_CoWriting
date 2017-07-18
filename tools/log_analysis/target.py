import pandas as pd
import seaborn as sns
import numpy as np
import os

from scipy.stats import ttest_ind,ttest_rel

p = "LOG_P_CSV"
s = "LOG_S_CSV"
r = "LOG_R_CSV"

file_subject = "withmeness-11-stdout"

def counts(condition):
    all_vals = []
    for i in range(60):
        try:
            name = condition + '/' + str(i) + '/' + file_subject + ".log"
            table = pd.read_csv(name, dtype=str, names=['msg','_','time','type','value'])
        except IOError:
            try:
                name = condition + '/' + str(i) + '/' + file_subject + str(i) + ".log"
                table = pd.read_csv(name, dtype=str, names=['msg','_','time','type','value'])
            except IOError:
                continue

        table = table[table["type"]=="target"]
        table = table["value"]
        table = table.as_matrix()[:550]
        score = np.array(table!="/away",dtype=float)
        all_vals.append(score)

    return np.stack(all_vals)

all_vals_r = counts(r)
all_vals_s = counts(s)
all_vals_p = counts(p)
sns.tsplot(all_vals_p,color='b')
sns.tsplot(all_vals_s,color='g')
sns.tsplot(all_vals_r,color='r')
sns.plt.show()
