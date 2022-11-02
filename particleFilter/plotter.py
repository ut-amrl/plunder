import matplotlib.pyplot as plt
import numpy as np 
import csv
import sys
import os
from matplotlib.colors import ListedColormap

# Initialization
trajectories = []
velocities = []
positions = []

gtTrajectory = []
gtLA = []
gtVelocity = []
gtPosition = []

settings = {}

# ----- I/O ---------------------------------------------=

# Reads in csv files
def readTrajectories(inPath):
    inFile = open(inPath, "r")
    reader = csv.reader(inFile)
    for x in inFile:
        traj = []
        for elem in next(reader):
            traj.append(float(elem))
        trajectories.append(traj)
    return

def readGroundTruth(gtPath):
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

def plotTraj(inF, outP, gtF):
    global trajectories, velocities, positions, gtTrajectory, gtLA, gtVelocity, gtPosition

    for iter in range(0, int(settings["numIterations"])):
        for robot in range(0, int(settings["numRobots"])):
            
            inFile = inF + str(iter) + "-" + str(robot) + ".csv"
            outPath = outP + str(iter) + "-" + str(robot) + "-"
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

            maxTime = min(int(settings["timeStepsPlot"]), min(len(gtTrajectory), len(trajectories[0])))
            particlesPlotted = min(int(settings["particlesPlotted"]), len(trajectories))

            # Calculate full state sequences
            runSimulation()

            # Acceleration
            fig, (ax1, ax1b, ax2, ax3) = plt.subplots(4, gridspec_kw={'height_ratios': [3, 5, 1, 1]})
            fig.suptitle('hi-level actions vs. time')

            freq = 1
            actions = [[0] * maxTime, [0] * maxTime, [0] * maxTime]
            gt = [[0] * maxTime, [0] * maxTime, [0] * maxTime]
            times = []
            for t in range(0, maxTime):
                times.append(t)
                if t % freq == 0:
                    for i in range(0, len(trajectories)):
                        a = trajectories[i][t]
                        if a > 0:
                            actions[0][t] += 1
                        elif a == 0:
                            actions[1][t] += 1
                        elif a < 0:
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
            
            sum = np.sum(actions, axis=0)
            actions = np.multiply(np.divide(actions, sum), 100)
            
            ax1.bar(times, actions[0], width=1, bottom=np.add(actions[2], actions[1]), color="#05a655", label="ACC")
            ax1.bar(times, actions[1], width=1, bottom=actions[2], color="#f8ff99", label="CON")
            ax1.bar(times, actions[2], width=1, color="#ff6040", label="DEC")
            color_graph = []
            for i in range(particlesPlotted):
                one_row = []
                for j in range(len(trajectories[0])):
                    c = 1
                    if (trajectories[i][j]<0):
                        c=0
                    elif (trajectories[i][j]>0):
                        c=2
                    one_row.append(c)
                color_graph.append(one_row)
            ax1b.imshow(np.array(color_graph), cmap=ListedColormap(["red", "yellow", "green"]), origin="lower", vmin=0)
            
            ax2.bar(times, gt[0], color="#05a655", width=1)
            ax2.bar(times, gt[1], color="#f8ff99", width=1)
            ax2.bar(times, gt[2], color="#ff6040", width=1)

            ax3.bar(times, gtLA, color="red", width=1)
            
            plt.xlabel('time')
            ax1.set_ylabel('hi-level action percentages')
            ax2.set_ylabel('ground\ntruth')

            # ax1.legend(loc="upper left")

            plt.show()
            plt.savefig(outPath + "accel.png")

            plt.clf()

            # Velocity
            for i in range(0, particlesPlotted):
                vel = velocities[i]
                plt.plot(vel[:maxTime], alpha=0.5, linestyle="dashed") 
            plt.plot(gtVelocity[:maxTime], alpha=1.0, label="ground truth")
                
            plt.xlabel('time') 
            plt.ylabel('velocity') 
            plt.title('velocity-time') 
            
            plt.legend(loc="upper left")

            plt.show()
            plt.savefig(outPath + "vel.png")

            plt.clf()

            # Position
            for i in range(0, particlesPlotted):
                pos = positions[i]
                plt.plot(pos[:maxTime], alpha=0.5, linestyle="dashed") 
            plt.plot(gtPosition[:maxTime], alpha=1.0, label="ground truth")
                
            plt.xlabel('time') 
            plt.ylabel('position') 
            plt.title('position-time')

            plt.legend(loc="upper left") 
            
            plt.show()
            plt.savefig(outPath + "pos.png")

            plt.close('all')

    return
    
# ----- Main ---------------------------------------------

def main():
    global trajectories, velocities, positions, gtTrajectory, gtLA, gtVelocity, gtPosition

    # Read settings
    readSettings("settings.txt")
    
    # I/O
    pureInFile = settings["altPath"]
    pfInFile = settings["trajGenPath"]

    gtFile = settings["stateGenPath"]

    pureOutPath = settings["plotGenPath"]+"pure/"
    pfOutPath = settings["plotGenPath"]+"pf/"

    try:
        plotTraj(pfInFile, pfOutPath, gtFile)
    except Exception as e: print(e)

    try:
        plotTraj(pureInFile, pureOutPath, gtFile)
    except Exception as e: print(e)
    

if __name__ == "__main__":
    main()