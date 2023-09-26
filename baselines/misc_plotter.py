import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({'font.size': 14})

target = "stack"

# 1D-target
if target == "1D-target":
    training = [-148951, -129579, -127934, -110743, -103292, -101026, -98482, -97827.3, -96767.3, -97076.3, -97271.7, -97197.6, -96927.7, -97026.7, -97470.9, -96666.7]
    testing = [-428431, -361798, -358302, -365852, -308766, -302923, -296427, -295278, -293824, -294168, -294679, -293654, -293072, -293289, -295898, -293641]
    training = [each / (100*125*10) for each in training]
    testing = [each / (100*125*30) for each in testing]
    x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
    ren = range(0, 16, 3)

# 2D-highway
if target == "2D-highway":
    training = [-7.57595,-1.12849,-0.287779,0.162089,0.261831,0.277573,0.321149,0.259748,0.390868,0.367335,0.387768,0.392703]
    testing = [-6.73988,-0.4518745,0.0126962,0.0455108,0.3429345,0.292507,0.345308,0.3075035,0.3234745,0.3355485,0.378921,0.3667095]
    x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    ren = range(0, 13, 3)

# 2D-merge
if target == "2D-merge":
    training = [-4.0354,-2.45907,-1.90358,-1.65009,-1.52427,-1.46253,-1.25326,-0.942493,-0.854028,-0.809816,-0.783505,-0.780442,-0.759502,-0.797437,-0.780687,-0.762582]
    testing = [-3.98208,-2.64202,-2.13013,-1.87573,-1.70812,-1.64285,-1.33789,-1.05342,-0.966207,-0.918407,-0.894232,-0.895988,-0.9115,-0.894763,-0.885708,-0.891118]
    x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
    ren = range(0, 16, 3)

# Pick-and-place
if target == "pick-and-place":
    training = [-4.6289,-2.64225,-2.23912,-2.51727,-2.46644,-1.72885,-1.76077,-1.99714,-2.07455]
    testing = [-3.08528,-1.99949,-1.64265,-1.55045,-1.51385,-1.24738,-1.27035,-1.33797,-1.35887]
    x = [1, 2, 3, 4, 5, 6, 7, 8, 9]
    ren = range(0, 10, 2)

# Stack
if target == "stack":
    training = [-11.1842,-2.90982,-2.00568,-1.48507,-1.41652,-1.20787,-1.42555,-1.40436]
    testing = [-11.4213,-2.75336,-1.62262,-1.28502,-1.20091,-0.962306,-0.99673,-1.12744]
    x = [1, 2, 3, 4, 5, 6, 7, 8]
    ren = range(0, 8, 2)

fig, ax = plt.subplots()
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

ax.plot(x, training, label = "training set", linewidth=2)
ax.plot(x, testing, label = "test set", linewidth=2)

plt.xlabel("Iteration")
plt.xticks(ren)
plt.ylabel("Log Obs. Likelihood")

plt.legend(loc = 'lower right')
plt.tight_layout()
plt.grid(linestyle='dotted')
plt.savefig(target + "-emloop.png", dpi=1200)
plt.show()

# LDIPS tests

# # 1D-target
# testing = [-1.10372e+06, -469523, -465382, -465691, -465691, -465691, -465691, -465691, -465691]
# emdips = [-1.10372e+06, -428431, -361798, -358302, -365852, -308766, -302923, -296427, -295278]
# testing = [each / (100*125*30) for each in testing]
# emdips = [each / (100*125*30) for each in emdips]


# x = [0, 1, 2, 3, 4, 5, 6, 7, 8]

# fig, ax = plt.subplots()
# ax.spines['top'].set_visible(False)
# ax.spines['right'].set_visible(False)

# ax.plot(x, testing, label = "deterministic")
# ax.plot(x, emdips, label = "probabilistic")

# plt.xlabel("Iteration")
# plt.ylabel("Average Obs Likelihood\n(log scale)")

# plt.legend(loc = 'lower right')
# plt.grid()
# plt.savefig("1D-target-ldips.png", dpi=1200)
# plt.show()