import matplotlib.pyplot as plt
import numpy as np 
import csv
import sys

# I/O
inFile = open("particleFilter/out/pf.csv", "r")
gtFile = open("accSim/out/data.csv", "r")
outFile1 = "particleFilter/plots/accel.png"
outFile2 = "particleFilter/plots/velocity.png"
outFile3 = "particleFilter/plots/position.png"

reader = csv.reader(inFile)
gtReader = csv.reader(gtFile)

# Default configuration
max_t = 1000
printed_particles = 10

# Initialization
trajectories = []
velocities = []
positions = []

gtTrajectory = []
gtVelocity = []
gtPosition = []

# Reads in csv files
def readTrajectories():
    for x in inFile:
        traj = []
        for elem in next(reader):
            traj.append(float(elem))
        trajectories.append(traj)
    return

def readGroundTruth():
    next(gtReader)

    for info in gtReader:
        gtPosition.append(float(info[1]))
        gtVelocity.append(float(info[2]))
        gtTrajectory.append(info[4].strip())
    return

# Populates velocity and position values based on acceleration data
def fillVelocity(acc, dt):
    vel = []
    curVel = 0
    for val in acc:
        curVel += val * dt
        vel.append(curVel)
    return vel

def fillPosition(vel, dt):
    pos = []
    curPos = 0
    lastVel = 0
    for v in vel:
        curPos += 0.5*(lastVel + v)*dt
        lastVel = v
        pos.append(curPos)
    return pos

def runSimulation():
    for traj in trajectories:
        velocities.append(fillVelocity(traj, 0.1))
    
    for vel in velocities:
        positions.append(fillPosition(vel, 0.1))
    
    return

# --------------------------------------------------------------------------------------------
def main():
    global max_t, printed_particles
    # Read parameters
    if len(sys.argv) > 1:
        if len(sys.argv) < 3:
            print("Please run in the following manner: python3 plotter.py <max time steps> <particles to plot>")
            exit()
        max_t = int(sys.argv[1])
        printed_particles = int(sys.argv[2])

    # I/O
    readTrajectories()
    readGroundTruth()

    max_t = min(max_t, min(len(gtTrajectory), len(trajectories[0])))
    printed_particles = min(printed_particles, len(trajectories))

    # Calculate full state sequences
    runSimulation()

    # Acceleration
    fig, (ax1, ax2) = plt.subplots(2, gridspec_kw={'height_ratios': [5, 1]})
    fig.suptitle('low-level actions vs. time')
    # for i in range(0, min(printed_particles, len(trajectories))):
    #     traj = trajectories[i]
    #     ax1.plot(traj[:min(len(traj), max_t)], alpha=0.1, linestyle="none", marker="o", color='b') 
    # ax2.plot(gtTrajectory[:min(len(gtTrajectory), max_t)], alpha=0.5, linestyle="none", marker="o", color='b')
    # plt.xlabel('time') 
    # plt.ylabel('acceleration')

    freq = 2
    actions = [[0] * max_t, [0] * max_t, [0] * max_t]
    gt = [[0] * max_t, [0] * max_t, [0] * max_t]
    times = []
    for t in range(0, max_t):
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

    ax1.bar(times, actions[0], width=1, bottom=np.add(actions[2], actions[1]), color="#05a655", label="ACC")
    ax1.bar(times, actions[1], width=1, bottom=actions[2], color="#f8ff99", label="CON")
    ax1.bar(times, actions[2], width=1, color="#ff6040", label="DEC")
    
    ax2.bar(times, gt[0], color="#05a655", width=1)
    ax2.bar(times, gt[1], color="#f8ff99", width=1)
    ax2.bar(times, gt[2], color="#ff6040", width=1)
    
    plt.xlabel('time')
    ax1.set_ylabel('low-level action counts')
    ax2.set_ylabel('ground\ntruth')

    ax1.legend(loc="upper left")

    plt.show()
    plt.savefig(outFile1)

    plt.clf()

    # Velocity
    for i in range(0, printed_particles):
        vel = velocities[i]
        plt.plot(vel[:max_t], alpha=0.5, linestyle="dashed") 
    plt.plot(gtVelocity[:max_t], alpha=1.0, label="ground truth")
        
    plt.xlabel('time') 
    plt.ylabel('velocity') 
    plt.title('velocity-time') 
    
    plt.legend(loc="upper left")

    plt.show()
    plt.savefig(outFile2)

    plt.clf()

    # Position
    for i in range(0, printed_particles):
        pos = positions[i]
        plt.plot(pos[:max_t], alpha=0.5, linestyle="dashed") 
    plt.plot(gtPosition[:max_t], alpha=1.0, label="ground truth")
        
    plt.xlabel('time') 
    plt.ylabel('position') 
    plt.title('position-time')

    plt.legend(loc="upper left") 
    
    plt.show()
    plt.savefig(outFile3)
    return

if __name__ == "__main__":
    main()