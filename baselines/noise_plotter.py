import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({'font.size': 14})

# Noise experiments

# 2D-merge
gt = [0.414148, 0.314877, -0.031643, -0.687255]
emsynth = [0.395291, 0.304421, -0.0966433, -0.885708]
stable = [0.395291, 0.150416, -0.554026, -1.31767]
greedy = [0.302961, 0.0192867, -0.875729, -2.40832]
bcplus = [0.2648504738, 0.1713373716, -0.2398012849, -1.019744255]
bc = [0.1627800541, -0.008859962692, -0.445538476, -1.17547465]
ldips = [0.122663, 0.00918271, -0.467166, -1.24605]
gail = [0, 0, 0, 0] # TODO

x = [0.25, 0.5, 0.75, 1]
fig, ax = plt.subplots()
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

ax.plot(x, np.subtract(emsynth, gt), "*", linestyle='solid', label = "PLUNDER", linewidth=2)
ax.plot(x, np.subtract(greedy, gt), "^", linestyle='dotted', label = "FitGreedy",linewidth=2)
ax.plot(x, np.subtract(stable, gt), "v", linestyle='dotted', label = "FitSmooth", linewidth=2)
ax.plot(x, np.subtract(bc, gt), "x", linestyle='dotted', label = "BC", linewidth=2)
ax.plot(x, np.subtract(bcplus, gt), "+", linestyle='dotted', label = "BC+", linewidth=2)
ax.plot(x, np.subtract(ldips, gt), "+", linestyle='dotted', label = "LDIPS", linewidth=2)
# ax.plot(x, np.subtract(gail, gt), "+", linestyle='dotted', label = "GAIL", linewidth=2)

plt.xlabel("Noise Level")
plt.ylabel("Log Likelihood (Relative to GT)")
plt.xticks([0.25, 0.5, 0.75, 1])

plt.legend(loc = 'lower left')
plt.tight_layout()
plt.grid(linestyle='dotted')
plt.savefig("noise.png", dpi=2000)
plt.show()