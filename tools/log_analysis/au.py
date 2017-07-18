import pandas as pd
import seaborn as sns
import numpy as np
import os

from scipy.stats import ttest_ind,ttest_rel

p = "LOG_P_CSV"
s = "LOG_S_CSV"
r = "LOG_R_CSV"

file_subject = "features_extractor-2-stdout"


def counts(condition):
    all_vals = []
    sums = []
    for i in range(60):
        try:
            name = condition + '/' + str(i) + '/' + file_subject + ".log"
            table = pd.read_csv(name, dtype=str, names=['_','msg','time','_','au','_','value','_'])
        except IOError:
            try:
                name = condition + '/' + str(i) + '/' + file_subject + str(i) + ".log"
                table = pd.read_csv(name, dtype=str, names=['_','msg','time','_','au','_','value','_'])
            except IOError:
                continue

        # 6+12 hapiness
        # 1+2+5B+26 surprise
        #  	1+4+15 sad
        # 4+5+7+23 anger
        # 9+15+16 disgust
        #  	R12A+R14A contemp

        table_6 = table[table['au']=='AU06']
        table_12 = table[table['au']=='AU12']
        vals_6 = table_6['value']
        #vals_6 = pd.to_numeric(vals_6)
        vals_6 = vals_6.as_matrix()[:550]
        for i in range(len(vals_6)):
            vals_6[i] = float(vals_6[i][:-6])
        vals_12 = table_12['value']
        vals_12 = vals_12.as_matrix()[:550]
        for i in range(len(vals_12)):
            vals_12[i] = float(vals_12[i][:-6])
        all_vals.append(vals_6 + vals_12)
        sums.append(np.sum(vals_6 + vals_12))
        '''
        table_4 = table[table['au']=='AU04']
        table_7 = table[table['au']=='AU07']
        table_5 = table[table['au']=='AU05']
        table_23 = table[table['au']=='AU23']
        vals_4 = table_4['value']
        vals_7 = table_7['value']
        vals_5 = table_5['value']
        vals_23 = table_23['value']
        vals_4 = vals_4.as_matrix()[:550]
        vals_7 = vals_7.as_matrix()[:550]
        vals_5 = vals_5.as_matrix()[:550]
        vals_23 = vals_23.as_matrix()[:550]
        for i in range(len(vals_4)):
            vals_4[i] = float(vals_4[i][:-6])
        for i in range(len(vals_7)):
            vals_7[i] = float(vals_7[i][:-6])
        for i in range(len(vals_5)):
            vals_5[i] = float(vals_5[i][:-6])
        for i in range(len(vals_23)):
            vals_23[i] = float(vals_23[i][:-6])

        all_vals.append(vals_4+vals_7+vals_5+vals_23)
        '''

    return np.stack(all_vals), np.array(sums)

all_vals_r,sums_r = counts(r)
print('r done..')
all_vals_s,sums_s = counts(s)
print('s done..')
all_vals_p,sums_p  = counts(p)
print('p done..')

print(ttest_ind(sums_p,sums_s))
print(ttest_ind(sums_p,sums_r))
print(ttest_ind(sums_s,sums_r))

sns.tsplot(all_vals_r,color='g')
sns.tsplot(all_vals_p,color='b')
sns.tsplot(all_vals_s,color='r')
sns.plt.show()
