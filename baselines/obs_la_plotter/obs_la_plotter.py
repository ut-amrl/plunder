import matplotlib.pyplot as plt
import numpy as np 
import csv
import os
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
import time
plt.rcParams.update({'font.size': 14})

setting = "SS"

# Initialization
trajectories = []
gtLA = []
useGT = True

# Default colors
colors = ["#F08080", "#9FE2BF"]

# ----- I/O ---------------------------------------------=

# Reads in csv files (float)
def readTrajectoriesFloat(inPath):
    while not os.path.exists(inPath):
        time.sleep(1)

    inFile = open(inPath, "r")
    reader = csv.reader(inFile)
    for x in inFile:
        traj = []
        for elem in next(reader):
            traj.append(float(elem))
        trajectories.append(traj)
    return

def readLA(gtPath):
    global gtLA

    gtReader = csv.reader(open(gtPath, "r"))
    title_row = next(gtReader)
    title_row = [var.strip() for var in title_row]

    la_indices = []
    la_names = []
    for i in range(len(title_row)):
        if (title_row[i])[0:3] == "LA.":
            la_indices.append(i)
            la_names.append((title_row[i].strip())[3:])

    gtLA = []
    for i in range(len(la_indices)):
        gtLA.append([])

    for info in gtReader:
        for i in range(len(la_indices)):
            gtLA[i].append(float(info[la_indices[i]]))

    return la_names # Return names of low-level actions

# Plots low-level actions
def plotSingleLA(inF, outP, gtFile, iter, robot):
    global trajectories, gtLA
    
    la_names = readLA(gtFile)
    outPath = outP + "-"

    fig, axs = plt.subplots(4, figsize=(10, 10))
    fig.suptitle("Low-level actions")
    plt.xlabel('Time (s)')

    color_count = 0
    for name in range(len(la_names)):
        handles = []

        label_name = la_names[name]
        
        trajectories = []
        inFile = inF + "-" + la_names[name] + ".csv"
        readTrajectoriesFloat(inFile)

        PARTICLES_PLOTTED = min(100, len(trajectories))    
        
        maxTime = min(500, len(gtLA[0]))
        for arr in gtLA:
            del arr[maxTime:]
        for arr in trajectories:
            del arr[maxTime:]

        times = []
        for t in range(maxTime):
            times.append(t)

        axs[name].set_ylabel(label_name)
        axs[name].plot(times, gtLA[name], color=colors[0], linewidth=1.5)
        handles.append(Line2D([0], [0], label='actual', color=colors[0]))
        handles.append(Line2D([0], [0], label='predicted', color=colors[1]))
        color_count += 1

        for p in range(PARTICLES_PLOTTED):
            axs[name].plot(times, trajectories[p], color=colors[1], alpha=(1/PARTICLES_PLOTTED), linewidth=2.5)
        color_count += 1

        axs[name].grid(linestyle='dotted')
        # axs[name].legend(handles=handles, loc='upper right')

        for tick in axs[name].xaxis.get_major_ticks():
            tick.label.set_fontsize(15)
        for tick in axs[name].yaxis.get_major_ticks():
            tick.label.set_fontsize(15)

    fig.tight_layout()
    plt.grid(linestyle='dotted')
    plt.show()
    plt.savefig(outPath + "graph.png", dpi=2000)

    plt.clf()
    plt.close('all')

# ----- Main ---------------------------------------------

def main():
    global trajectories, gtLA

    plotSingleLA(setting, setting, setting + ".csv", 20, 24)

if __name__ == "__main__":
    main()