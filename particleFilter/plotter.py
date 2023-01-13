import matplotlib.pyplot as plt
import numpy as np 
import csv
import sys
import os
from matplotlib.colors import ListedColormap
import time

# Initialization
trajectories = []
gtTrajectory = []
gtLA = []

useGT = True

settings = {}

# Default colors
colors = ["#F08080", "#9FE2BF", "#6495ED", "#FFBF00", "#CCCCFF", "#40E0D0", "#DE3163", "#DFFF00", "#34495E", "#FF7F50"]

# ----- I/O ---------------------------------------------=

# Reads in csv files
def readTrajectories(inPath):
    while not os.path.exists(inPath):
        time.sleep(1)

    inFile = open(inPath, "r")
    reader = csv.reader(inFile)
    for x in inFile:
        traj = []
        for elem in next(reader):
            traj.append(int(elem))
        trajectories.append(traj)
    return

def readGroundTruth(gtPath):
    global useGT

    while not os.path.exists(gtPath):
        time.sleep(1)
    gtReader = csv.reader(open(gtPath, "r"))
    title_row = next(gtReader)

    ha_index = -1
    for i in range(0, len(title_row)):
        if title_row[i].strip() == "HA":
            ha_index = i

    if ha_index == -1:
        useGT = False
        return
    
    for info in gtReader:
        if len(info)>0:
            gtTrajectory.append(int(info[ha_index].strip()))

    return

def readLA(gtPath):
    global gtLA

    gtReader = csv.reader(open(gtPath, "r"))
    title_row = next(gtReader)
    title_row = [var.strip() for var in title_row]

    la_indices = []
    la_names = []
    for i in range(0, len(title_row)):
        if (title_row[i])[0:3] == "LA.":
            la_indices.append(i)
            la_names.append((title_row[i].strip())[3:])

    gtLA = []
    for i in range(0, len(la_indices)):
        gtLA.append([])

    for info in gtReader:
        for i in range(0, len(la_indices)):
            gtLA[i].append(float(info[la_indices[i]]))

    return la_names # Return names of low-level actions

def readSettings(sgPath):
    with open(sgPath) as f:
        lines = f.readlines()
        for line in lines:
            line = line.strip()
            vals = line.split()
            settings[vals[0]] = vals[1]
    return


# ----- Handles the main plotting logic ---------------------------------------------
def figureHandler(outP, actions, gt, color_graph, title, iter, robot, useGT):
    global trajectories, gtTrajectory
    outPath = outP + str(iter) + "-" + str(robot) + "-"

    maxTime = min(int(settings["PLOT_TIME"]), len(trajectories[0]))
    if useGT:
        maxTime = min(maxTime, len(gtTrajectory))
    PARTICLES_PLOTTED = min(int(settings["PARTICLES_PLOTTED"]), len(trajectories))

    for traj in trajectories:
        traj = traj[0:maxTime]
    gtTrajectory = gtTrajectory[0:maxTime]

    times = []
    for t in range(0, maxTime):
        times.append(t)

    if useGT:
        fig, (ax1, ax1b, ax2) = plt.subplots(3, gridspec_kw={'height_ratios': [4, 4, 1]})
        ax1.margins(0)
        ax1b.margins(0)
        ax2.margins(0)
    else:
        fig, (ax1, ax1b) = plt.subplots(2, gridspec_kw={'height_ratios': [1, 1]})
        ax1.margins(0)
        ax1b.margins(0)
    
    fig.suptitle(title)

    actions = np.multiply(np.nan_to_num(np.divide(actions, np.sum(actions, axis=0))), 100)

    cum_sum = [0.0] * maxTime
    for i in range(0, len(actions)):
        ax1.bar(times, actions[i], width=1, bottom=cum_sum, color=colors[i])
        cum_sum = np.add(cum_sum, actions[i])

    # TODO: color map is just using whatever order it wants. I can't figure out how to force it to use our desired order
    ax1b.imshow(np.array(color_graph), cmap=ListedColormap(colors[0:len(actions)]), origin="lower", vmin=0, aspect='auto', interpolation='none')

    plt.xlabel('time')
    ax1.set_ylabel('hi-level actions')
    ax1b.set_ylabel('hi-level actions\n(unsorted)')

    if useGT:
        for i in range(0, len(gt)):
            ax2.bar(times, gt[i], color=colors[i], width=1)
        ax2.set_ylabel('demo')

    fig.tight_layout()  
    plt.show()
    if os.path.exists(outPath[:outPath.rfind('/')]):
        plt.savefig(outPath + "graph.png")
    else:
        raise Exception('Plots folder deleted... restarting plotter')

    plt.clf()
    plt.close('all')

    return (actions, color_graph)

# Processes input and passes to figureHandler
def plotSingle(inF, outP, gtF, title, iter, robot):
    global trajectories, gtTrajectory

    inFile = inF + str(iter) + "-" + str(robot) + ".csv"
    gtFile = gtF + str(robot) + ".csv"

    trajectories = []
    gtTrajectory = []

    readTrajectories(inFile)
    readGroundTruth(gtFile)

    maxTime = min(int(settings["PLOT_TIME"]), len(trajectories[0]))
    if useGT:
        maxTime = min(maxTime, len(gtTrajectory))

    PARTICLES_PLOTTED = min(int(settings["PARTICLES_PLOTTED"]), len(trajectories))

    max_action = 0
    for t in range(0, maxTime):
        for i in range(0, len(trajectories)):
            max_action = max(max_action, trajectories[i][t])
        if useGT:
            max_action = max(max_action, gtTrajectory[t])
    
    actions = []
    gt = []
    for i in range(0, max_action + 1):
        actions.append([0] * maxTime)
        gt.append([0] * maxTime)
         
    for t in range(0, maxTime):
        for i in range(0, len(trajectories)):
            a = trajectories[i][t]
            actions[a][t] += 1
        if useGT:
            a = gtTrajectory[t]
            gt[a][t] += 1
    
    color_graph = []
    for i in range(PARTICLES_PLOTTED):
        one_row = []
        for j in range(maxTime):
            one_row.append(trajectories[i][j])
        color_graph.append(one_row)

    return figureHandler(outP, actions, gt, color_graph, title, iter, robot, useGT)


def plotSingleTimestep(inF, outP, gtF, title, iter):
    for robot in range(0, int(settings["NUM_ROBOTS"])):
        plotSingle(inF, outP, gtF, title, iter, robot)

# Plots low-level actions
def plotLA(outP, gtF, robot):
    global gtLA

    gtFile = gtF + str(robot) + ".csv"
    la_names = readLA(gtFile)
    outPath = outP + "LA-" + str(robot) + "-"
    
    maxTime = min(int(settings["PLOT_TIME"]), len(gtLA[0]))
    for arr in gtLA:
        del arr[maxTime:]

    times = []
    for t in range(0, maxTime):
        times.append(t)

    fig, axs = plt.subplots(len(gtLA))
    fig.suptitle("Low-level actions")
    plt.xlabel('time')

    if len(gtLA) == 1:
        axs.bar(times, gtLA[0], color=colors[0], width=1)
        axs.set_ylabel(la_names[0])
    else:
        for i in range(0, len(gtLA)):
            axs[i].bar(times, gtLA[i], color=colors[i], width=1)
            axs[i].set_ylabel(la_names[i])

    fig.tight_layout()  
    plt.show()
    if os.path.exists(outPath[:outPath.rfind('/')]):
        plt.savefig(outPath + "graph.png")
    else:
        raise Exception('Plots folder deleted... restarting plotter')

    plt.clf()
    plt.close('all')
    
# ----- Main ---------------------------------------------

def main():
    global trajectories, gtTrajectory, gtLA

    while not os.path.exists("settings.txt"):
        time.sleep(1)
    print("Settings file found...")
    # Read settings
    readSettings("settings.txt")
    
    # I/O
    pureInFile = settings["PURE_TRAJ"]
    pfInFile = settings["PF_TRAJ"]
    gtFile = settings["SIM_DATA"]
    pureOutPath = settings["PLOT_PATH"]+"pure/"
    pfOutPath = settings["PLOT_PATH"]+"pf/"

    try:
        print("Plotting ground truth...")
        for robot in range(0, int(settings["NUM_ROBOTS"])):
            plotSingle(pureInFile, pureOutPath, gtFile, 'Ground Truth Robots', 'gt', robot)

        print("Plotting low-level actions...")
        for robot in range(0, int(settings["NUM_ROBOTS"])):
            plotLA(pureOutPath, gtFile, robot)

        graph1 =    {   'inF': pfInFile,
                        'outP': pfOutPath,
                        'gtF': gtFile,
                        'title': 'Particle filter outputs'
                    }
        graph2 =    {   'inF': pureInFile,
                        'outP': pureOutPath,
                        'gtF': gtFile,
                        'title': 'ASP Test Run'
                    }
            

        for iter in range(0, int(settings["NUM_ITER"])):
            print("Plotting particle filter graphs, iteration " + str(iter))
            plotSingleTimestep(graph1['inF'], graph1['outP'], graph1['gtF'], graph1['title'], iter)
            print("Plotting pure ASP graphs, iteration " + str(iter))
            plotSingleTimestep(graph2['inF'], graph2['outP'], graph2['gtF'], graph2['title'], iter)
        
    except Exception as e:
        print(e)
        pass

if __name__ == "__main__":
    main()
