import pandas as pd
import numpy as np
import math

in_files = "supervised_learning_la/merge-easy-data/"
out_files = "supervised_learning_la/merge-impossible-data/"
out_files2 = "supervised_learning_ha/merge-impossible-data/"

num_traj = 30

pv1_dev = 0.005
pv2_dev = 0.5

target_pv1_dev = 0.03
target_pv2_dev = 3

add_pv1_dev = math.sqrt(target_pv1_dev * target_pv1_dev - pv1_dev * pv1_dev)
add_pv2_dev = math.sqrt(target_pv2_dev * target_pv2_dev - pv2_dev * pv2_dev)

print(add_pv1_dev)
print(add_pv2_dev)

pv1 = "LA.steer"
pv2 = "LA.acc"

for traj in range(0, num_traj):
    df = pd.read_csv(in_files + "data" + str(traj) + ".csv", skipinitialspace=True)

    df[pv1] = df.apply(lambda row: row[pv1] + np.random.normal(0, add_pv1_dev), axis=1)
    df[pv2] = df.apply(lambda row: row[pv2] + np.random.normal(0, add_pv2_dev), axis=1)
    
    np.savetxt(out_files + "data" + str(traj) + ".csv", df, delimiter=", ", header=", ".join(df.columns.values), fmt='%s', comments='', encoding=None)
    np.savetxt(out_files2 + "data" + str(traj) + ".csv", df, delimiter=", ", header=", ".join(df.columns.values), fmt='%s', comments='', encoding=None)