import matplotlib.pyplot as plt
import numpy as np 
import csv
import sys
import os
from matplotlib.colors import ListedColormap
import time
import settings

# Default colors
colors = ["#F08080", "#9FE2BF", "#6495ED", "#FFBF00", "#CCCCFF", "#40E0D0", "#DE3163", "#DFFF00", "#34495E", "#FF7F50"]

# ----- Handles the main plotting logic for HA ---------------------------------------------
def plot(yhat):
    outPath = "plots/HA"
    
    iter = 0
    for robot_iter in range(settings.validation_set):
        maxTime = settings.sim_time - 2
        times = []
        for t in range(maxTime):
            times.append(t)

        actions = []
        for ha in range(settings.numHA):
            actions.append([])
            for t in range(iter, iter + maxTime):
                actions[ha].append(yhat[t][ha] * 100)
        iter = iter + maxTime

        fig, ax1 = plt.subplots(1)
        ax1.margins(0)
        fig.suptitle("Neural network outputs")

        cum_sum = [0.0] * maxTime
        for i in range(0, len(actions)):
            ax1.bar(times, actions[i], width=1, bottom=cum_sum, color=colors[i])
            cum_sum = np.add(cum_sum, actions[i])

        plt.xlabel('time')
        ax1.set_ylabel('hi-level actions')

        fig.tight_layout()  
        # plt.show()
        plt.savefig(outPath + "graph" + str(robot_iter) + ".png")

        plt.clf()
        plt.close('all')

# ----- Handles the main plotting logic for LA ---------------------------------------------
def plotLA(expected1, expected2, actual):
    outPath = "plots/LA"
    
    iter = 0
    for robot_iter in range(settings.validation_set):
        maxTime = settings.sim_time - 2
        times = []
        for t in range(maxTime):
            times.append(t)

        if not settings.pred_var1 == None:
            pyplot.plot(actual[iter:iter+maxTime, 0], label=settings.pred_var1)
            pyplot.plot(expected1[iter:iter+maxTime], label='model output')
            pyplot.legend()
            pyplot.savefig("plots/la1-" + str(robot_iter) + ".png")

        if not settings.pred_var2 == None:
            pyplot.plot(actual[iter:iter+maxTime, settings.numHA+1], label=settings.pred_var2)
            pyplot.plot(expected2[iter:iter+maxTime], label='model output')
            pyplot.legend()
            pyplot.savefig("plots/la2-" + str(robot_iter) + ".png")

        iter += maxTime