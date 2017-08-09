# coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt

#"~/Code/hexapod_common/hexapod_controller/traj.csv",
df = pd.read_csv(
    "~/ros_workspaces/hexapod_ik/traj.csv",
    sep=", ",
    header=None,
    names=["time", "a", "b", "c", "x", "y", "z"],
    index_col="time",
    engine='python'
)

df.plot()
plt.show()
# import csv
# import matplotlib.pyplot as plt
#
# plt.plot([1,2,3,4])
# plt.ylabel('some numbers')
# plt.show()
#
# with open('traj.csv', newline='') as csvfile:
#      = csv.reader(csvfile, delimiter=' ', quotechar='|')
#     for row in spamreader:
#         print(', '.join(row))