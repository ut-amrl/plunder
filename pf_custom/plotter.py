import matplotlib.pyplot as plt 
import csv
import sys

# I/O
inFile = open("pf_custom/out/pf.csv", "r")
gtFile = open("accSim/out/data.csv", "r")
outFile1 = "pf_custom/plots/accel.png"
outFile2 = "pf_custom/plots/velocity.png"
outFile3 = "pf_custom/plots/position.png"

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
        gtTrajectory.append(float(info[3]))
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

    # Calculate full state sequences
    runSimulation()

    # Acceleration
    fig, (ax1, ax2) = plt.subplots(2)
    fig.suptitle('acceleration-time')
    for i in range(0, min(printed_particles, len(trajectories))):
        traj = trajectories[i]
        ax1.plot(traj[:min(len(traj), max_t)], alpha=0.1, linestyle="none", marker="o", color='b') 
    ax2.plot(gtTrajectory[:min(len(gtTrajectory), max_t)], alpha=0.5, linestyle="none", marker="o", color='b')
        
    plt.xlabel('time') 
    plt.ylabel('acceleration')
        
    plt.show()
    plt.savefig(outFile1)

    plt.clf()

    # Velocity
    for i in range(0, min(printed_particles, len(trajectories))):
        vel = velocities[i]
        plt.plot(vel[:min(len(traj), max_t)], alpha=0.5, linestyle="dashed") 
    plt.plot(gtVelocity[:min(len(gtTrajectory), max_t)], alpha=1.0)
        
    plt.xlabel('time') 
    plt.ylabel('velocity') 
    plt.title('velocity-time') 
        
    plt.show()
    plt.savefig(outFile2)

    plt.clf()

    # Position
    for i in range(0, min(printed_particles, len(trajectories))):
        pos = positions[i]
        plt.plot(pos[:min(len(traj), max_t)], alpha=0.5, linestyle="dashed") 
    plt.plot(gtPosition[:min(len(gtTrajectory), max_t)], alpha=1.0)
        
    plt.xlabel('time') 
    plt.ylabel('position') 
    plt.title('position-time') 
    
    plt.show()
    plt.savefig(outFile3)
    return

if __name__ == "__main__":
    main()
