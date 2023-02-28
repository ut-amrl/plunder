import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({'font.size': 14})

# # 1D-target
training = [-148951, -129579, -127934, -110743, -103292, -101026, -98482, -97827.3, -96767.3, -97076.3, -97271.7, -97197.6, -96927.7, -97026.7, -97470.9, -96666.7]
testing = [-428431, -361798, -358302, -365852, -308766, -302923, -296427, -295278, -293824, -294168, -294679, -293654, -293072, -293289, -295898, -293641]
training = [each / (100*125*10) for each in training]
testing = [each / (100*125*30) for each in testing]
x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]


# training = [-989609, -110309, -1404.22, 5764.13, 7228.91, 5206.51, 4549.19, 40038.5, 36732.3, 37120.1, 42511.1]
# testing = [-3.09332e+06, -461588, -232312, -182830, -107630, -99303.8, -113750, -13528.5, -38673.7, -29055, -15200.1]
# training = [each / (50*100*10) for each in training]
# testing = [each / (50*100*30) for each in testing]
# x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]


fig, ax = plt.subplots()
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

ax.plot(x, training, label = "training set")
ax.plot(x, testing, label = "testing set")

plt.xlabel("Iteration")
plt.ylabel("Average Obs Likelihood (log scale)")

plt.legend(loc = 'lower right')
plt.tight_layout()
plt.savefig("1D-target-emloop.png", dpi=1200)
# plt.savefig("2D-highway-emloop.png", dpi=1200)
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
# plt.savefig("1D-target-ldips.png", dpi=1200)
# plt.show()


# Noise experiments

# 2D-merge
# gt = [1.238937333, 0.8958613333, 0.1214428, -1.346808]
# perfect = [0.9602453333, 0.2720666667, -0.2379098667, -2.088324]
# emsynth = [1.062952, 0.2565986667, -0.493004533, -1.868958667]
# stable = [0.7821733333, -0.653108, -0.993234666, -3.038394667]
# greedy = [0.2356574667, -0.7432653333, -2.749946667, -6.557186667]
# nn_ha = [0.6695968209, -1.295732139, -0.7622469043, -2.388837975]
# nn_la = [0.9564075951, -0.115898977, -1.109390473, -5.021243755]

# x = [0.25, 0.5, 0.75, 1]

# perfect = np.subtract(gt, perfect)
# emsynth = np.subtract(gt, emsynth)
# stable = np.subtract(gt, stable)
# greedy = np.subtract(gt, greedy)
# nn_ha = np.subtract(gt, nn_ha)
# nn_la = np.subtract(gt, nn_la)

# fig, ax = plt.subplots()
# ax.spines['top'].set_visible(False)
# ax.spines['right'].set_visible(False)

# ax.plot(x, perfect, ".", linestyle='solid', label = "EM-TRUTH")
# ax.plot(x, emsynth, "*", linestyle='--', label = "EM-SYNTH")
# ax.plot(x, greedy, "^", linestyle='dotted', label = "M-SYNTH")
# ax.plot(x, stable, "v", linestyle='dotted', label = "M-SYNTH+")
# ax.plot(x, nn_la, "x", linestyle='dotted', label = "Supervised")
# ax.plot(x, nn_ha, "+", linestyle='dotted', label = "Supervised+")

# plt.xlabel("Noise Level")
# plt.ylabel("Likelihood Difference\n(relative to ground truth)")

# plt.legend(loc = 'upper left')
# plt.savefig("noise_experiments.png", dpi=1200)
# plt.show()