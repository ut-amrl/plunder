# TODO: generalize plotter

import matplotlib.pyplot as plt
import numpy as np 
import csv
import sys
import os
from matplotlib.colors import ListedColormap
import time

# Initialization
trajectories = []
velocities = []
positions = []

gtTrajectory = []
gtLA = []
gtVelocity = []
gtPosition = []

settings = {}

settings_path=""
settings_mod_time=0

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
    next(gtReader)

    for info in gtReader:
        gtPosition.append(float(info[1]))
        gtVelocity.append(float(info[2]))
        gtLA.append(float(info[3]))
        gtTrajectory.append(info[4].strip())
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

# Populates velocity and position values based on acceleration data
def fillVelocity(acc, dt):
    vel = []
    curVel = 0
    vel.append(curVel)
    for val in acc:
        curVel += val * dt
        vel.append(curVel)
    return vel

def fillPosition(vel, dt):
    pos = []
    curPos = 0
    lastVel = 0
    pos.append(curPos)
    for v in vel:
        curPos += 0.5*(lastVel + v)*dt
        lastVel = v
        pos.append(curPos)
    return pos

def runSimulation():
    for traj in trajectories:
        velocities.append(fillVelocity(traj, float(settings["T_STEP"])))
    
    for vel in velocities:
        positions.append(fillPosition(vel, float(settings["T_STEP"])))
    
    return

def figureHandler(outP, actions, gt, color_graph, title, iter, robot, useGT):
    global trajectories, velocities, positions, gtTrajectory, gtLA, gtVelocity, gtPosition
    outPath = outP + str(iter) + "-" + str(robot) + "-"

    maxTime = min(int(settings["PLOT_TIME"]), min(len(gtTrajectory), len(trajectories[0])))
    PARTICLES_PLOTTED = min(int(settings["PARTICLES_PLOTTED"]), len(trajectories))

    for traj in trajectories:
        traj = traj[0:maxTime]
    velocities = velocities[0:maxTime]
    positions = positions[0:maxTime]
    gtTrajectory = gtTrajectory[0:maxTime]
    gtLA = gtLA[0:maxTime]
    gtVelocity = gtVelocity[0:maxTime]
    gtPosition = gtPosition[0:maxTime]

    times = []
    for t in range(0, maxTime):
        times.append(t)

    # Acceleration
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

    sum = np.sum(actions, axis=0)
    actions = np.multiply(np.nan_to_num(np.divide(actions, sum)), 100)

    ax1.bar(times, actions[0], width=1, bottom=np.add(actions[2], actions[1]), color="#05a655", label="ACC")
    ax1.bar(times, actions[1], width=1, bottom=actions[2], color="#f8ff99", label="CON")
    ax1.bar(times, actions[2], width=1, color="#ff6040", label="DEC")

    ax1b.imshow(np.array(color_graph), cmap=ListedColormap(["red", "yellow", "green"]), origin="lower", vmin=0, aspect='auto', interpolation='none')

    plt.xlabel('time')
    ax1.set_ylabel('hi-level actions')

    if useGT:
        ax2.bar(times, gtLA, color="red", width=1)

        ax3.bar(times, gt[0], color="#05a655", width=1)
        ax3.bar(times, gt[1], color="#f8ff99", width=1)
        ax3.bar(times, gt[2], color="#ff6040", width=1)

        ax2.set_ylabel('demo')
        ax3.set_ylabel('ground\ntruth')

    fig.tight_layout()  
    plt.show()
    if os.path.exists(outPath[:outPath.rfind('/')]):
        plt.savefig(outPath + "accel.png")
    else:
        raise Exception('Plots folder deleted... restarting plotter')

    plt.clf()
    plt.close('all')

    return (actions, color_graph)

def plotSingle(inF, outP, gtF, title, iter, robot):
    global trajectories, velocities, positions, gtTrajectory, gtLA, gtVelocity, gtPosition

    inFile = inF + str(iter) + "-" + str(robot) + ".csv"
    gtFile = gtF + str(robot) + ".csv"

    trajectories = []
    velocities = []
    positions = []

    gtTrajectory = []
    gtLA = []
    gtVelocity = []
    gtPosition = []

    readTrajectories(inFile)
    readGroundTruth(gtFile)

    maxTime = min(int(settings["PLOT_TIME"]), min(len(gtTrajectory), len(trajectories[0])))
    PARTICLES_PLOTTED = min(int(settings["PARTICLES_PLOTTED"]), len(trajectories))

    freq = 1
    actions = [[0] * maxTime, [0] * maxTime, [0] * maxTime]
    gt = [[0] * maxTime, [0] * maxTime, [0] * maxTime]
    for t in range(0, maxTime):
        if t % freq == 0:
            for i in range(0, len(trajectories)):
                a = trajectories[i][t]
                if a == 0: # ACC
                    actions[0][t] += 1
                elif a == 2: # CON
                    actions[1][t] += 1
                elif a == 1: # DEC
                    actions[2][t] += 1
            a = gtTrajectory[t]
            if a == "ACC":
                gt[0][t] += 1
            elif a == "CON":
                gt[1][t] += 1
            elif a == "DEC":
                gt[2][t] += 1
        else:
            actions[0][t] = actions[0][t-1]
            actions[1][t] = actions[1][t-1]
            actions[2][t] = actions[2][t-1]
            gt[0][t] = gt[0][t-1]
            gt[1][t] = gt[1][t-1]
            gt[2][t] = gt[2][t-1]
    
    color_graph = []
    for i in range(PARTICLES_PLOTTED):
        one_row = []
        for j in range(maxTime):
            c = 1 # CON
            if (trajectories[i][j] == 1): # DEC
                c=0
            elif (trajectories[i][j] == 0): # ACC
                c=2
            one_row.append(c)
        color_graph.append(one_row)
    
    # # Calculate full state sequences
    # runSimulation()

    # # Velocity
    # for i in range(0, PARTICLES_PLOTTED):
    #     vel = velocities[i]
    #     plt.plot(vel[:maxTime], alpha=0.5, linestyle="dashed") 
    # plt.plot(gtVelocity[:maxTime], alpha=1.0, label="ground truth")
        
    # plt.xlabel('time') 
    # plt.ylabel('velocity') 
    # plt.title('velocity-time') 
    
    # plt.legend(loc="upper left")

    # plt.show()
    # plt.savefig(outPath + "vel.png")

    # plt.clf()

    # # Position
    # for i in range(0, PARTICLES_PLOTTED):
    #     pos = positions[i]
    #     plt.plot(pos[:maxTime], alpha=0.5, linestyle="dashed") 
    # plt.plot(gtPosition[:maxTime], alpha=1.0, label="ground truth")
        
    # plt.xlabel('time') 
    # plt.ylabel('position') 
    # plt.title('position-time')

    # plt.legend(loc="upper left") 
    
    # plt.show()
    # plt.savefig(outPath + "pos.png")
    return figureHandler(outP, actions, gt, color_graph, title, iter, robot, True)


def plotSingleTimestep(inF, outP, gtF, title, iter):
    cum_actions = [[0] * int(settings["PLOT_TIME"]), [0] * int(settings["PLOT_TIME"]), [0] * int(settings["PLOT_TIME"])]
    cum_color_graph = []

    for robot in range(0, int(settings["NUM_ROBOTS"])):
        tup = plotSingle(inF, outP, gtF, title, iter, robot)

        for i in range(len(cum_actions)):
            for t in range(0, len(tup[0][i])):
                cum_actions[i][t] += tup[0][i][t]
        cum_color_graph += tup[1]

    figureHandler(outP, cum_actions, 0, cum_color_graph, title, iter, "cumulative", False)

def plotTrajectories(graph1, graph2):
    for iter in range(0, int(settings["NUM_ITER"])):
        plotSingleTimestep(graph1['inF'], graph1['outP'], graph1['gtF'], graph1['title'], iter)
        plotSingleTimestep(graph2['inF'], graph2['outP'], graph2['gtF'], graph2['title'], iter)
    return


    
# ----- Main ---------------------------------------------

def main():
    global trajectories, velocities, positions, gtTrajectory, gtLA, gtVelocity, gtPosition

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
    cum_actions = [[0] * int(settings["PLOT_TIME"]), [0] * int(settings["PLOT_TIME"]), [0] * int(settings["PLOT_TIME"])]
    cum_color_graph = []

    try:
        for robot in range(0, int(settings["NUM_ROBOTS"])):
            tup = plotSingle(pureInFile, pureOutPath, gtFile, 'Ground Truth Robots', 'gt', robot)

            for i in range(len(cum_actions)):
                for t in range(0, len(tup[0][i])):
                    cum_actions[i][t] += tup[0][i][t]
            cum_color_graph += tup[1]

        figureHandler(pureOutPath, cum_actions, 0, cum_color_graph, 'Ground Truth Robots', 'gt', "cumulative", False)


        graph1 = {  'inF': pfInFile,
                    'outP': pfOutPath,
                    'gtF': gtFile,
                    'title': 'Particle filter outputs'
                }
        graph2 = {  'inF': pureInFile,
                    'outP': pureOutPath,
                    'gtF': gtFile,
                    'title': 'ASP Test Run'
                }
        plotTrajectories(graph1, graph2)
    except Exception as e:
        print(e)
        pass

if __name__ == "__main__":
    main()
    # os.system("make plt")
