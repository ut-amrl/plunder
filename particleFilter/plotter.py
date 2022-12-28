# TODO: deal with LA

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

settings = {}

settings_path=""
settings_mod_time=0

colors = ["#F08080", "#9FE2BF", "#6495ED", "#FFBF00", "#CCCCFF", "#40E0D0", "#DE3163", "#DFFF00", "#34495E", "#FF7F50"]

# ----- I/O ---------------------------------------------=

# Reads in csv files
def readTrajectories(inPath):
    while not os.path.exists(inPath):
        time.sleep(1)
        # if not os.path.exists(settings_path) or os.path.getmtime(settings_path)!=settings_mod_time:
        #     raise Exception('Settings file changed... restarting plotter')

    inFile = open(inPath, "r")
    reader = csv.reader(inFile)
    for x in inFile:
        traj = []
        for elem in next(reader):
            traj.append(int(elem))
        trajectories.append(traj)
    return

def readGroundTruth(gtPath):
    while not os.path.exists(gtPath):
        time.sleep(1)
        # if not os.path.exists(settings_path) or os.path.getmtime(settings_path)!=settings_mod_time:
        #     raise Exception('Settings file changed... restarting plotter')
    gtReader = csv.reader(open(gtPath, "r"))
    title_row = next(gtReader)

    for i in range(0, len(title_row)):
        if title_row[i].strip() == "HA":
            ha_index = i

    for info in gtReader:
        gtLA.append(float(info[3]))
        gtTrajectory.append(int(info[ha_index].strip()))

    return

def readSettings(sgPath):
    with open(sgPath) as f:
        settings_path = sgPath
        settings_mod_time = os.path.getmtime(sgPath)
        lines = f.readlines()
        for line in lines:
            line = line.strip()
            vals = line.split()
            settings[vals[0]] = vals[1]
    return


# ----- Simulation ---------------------------------------------
def figureHandler(outP, actions, gt, color_graph, title, iter, robot, useGT):
    global trajectories, gtTrajectory, gtLA
    outPath = outP + str(iter) + "-" + str(robot) + "-"

    maxTime = min(int(settings["PLOT_TIME"]), min(len(gtTrajectory), len(trajectories[0])))
    PARTICLES_PLOTTED = min(int(settings["PARTICLES_PLOTTED"]), len(trajectories))

    for traj in trajectories:
        traj = traj[0:maxTime]
    gtTrajectory = gtTrajectory[0:maxTime]
    gtLA = gtLA[0:maxTime]

    times = []
    for t in range(0, maxTime):
        times.append(t)

    # Low-level actions
    if useGT:
        fig, (ax1, ax1b, ax2, ax3) = plt.subplots(4, gridspec_kw={'height_ratios': [4, 4, 1, 1]})
        ax1.margins(0)
        ax1b.margins(0)
        ax2.margins(0)
        ax3.margins(0)
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

    ax1b.imshow(np.array(color_graph), cmap=ListedColormap(colors[0:len(actions)]), origin="lower", vmin=0, aspect='auto', interpolation='none')

    plt.xlabel('time')
    ax1.set_ylabel('hi-level actions')

    if useGT:
        ax2.bar(times, gtLA, color="red", width=1)

        for i in range(0, len(gt)):
            ax3.bar(times, gt[i], color=colors[i], width=1)

        ax2.set_ylabel('demo')
        ax3.set_ylabel('ground\ntruth')

    fig.tight_layout()  
    plt.show()
    if os.path.exists(outPath[:outPath.rfind('/')]):
        plt.savefig(outPath + "graph.png")
    else:
        raise Exception('Plots folder deleted... restarting plotter')

    plt.clf()
    plt.close('all')

    return (actions, color_graph)

def plotSingle(inF, outP, gtF, title, iter, robot):
    global trajectories, gtTrajectory, gtLA

    inFile = inF + str(iter) + "-" + str(robot) + ".csv"
    gtFile = gtF + str(robot) + ".csv"

    trajectories = []
    gtTrajectory = []
    gtLA = []

    readTrajectories(inFile)
    readGroundTruth(gtFile)

    maxTime = min(int(settings["PLOT_TIME"]), min(len(gtTrajectory), len(trajectories[0])))
    PARTICLES_PLOTTED = min(int(settings["PARTICLES_PLOTTED"]), len(trajectories))

    max_action = 0
    for t in range(0, maxTime):
        for i in range(0, len(trajectories)):
            max_action = max(max_action, trajectories[i][t])
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
        a = gtTrajectory[t]
        gt[a][t] += 1
    
    color_graph = []
    for i in range(PARTICLES_PLOTTED):
        one_row = []
        for j in range(maxTime):
            one_row.append(trajectories[i][j])
        color_graph.append(one_row)

    return figureHandler(outP, actions, gt, color_graph, title, iter, robot, True)


def plotSingleTimestep(inF, outP, gtF, title, iter):
    for robot in range(0, int(settings["NUM_ROBOTS"])):
        tup = plotSingle(inF, outP, gtF, title, iter, robot)


    
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

    # Plot ground truth

    try:
        print("Plotting ground truth...")
        for robot in range(0, int(settings["NUM_ROBOTS"])):
            tup = plotSingle(pureInFile, pureOutPath, gtFile, 'Ground Truth Robots', 'gt', robot)

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
