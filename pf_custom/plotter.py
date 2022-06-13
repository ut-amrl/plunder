import matplotlib.pyplot as plt 
import csv
    
inFile = open("pf.csv", "r")
gtFile = open("../accSim/data.csv", "r")
reader = csv.reader(inFile)
gtReader = csv.reader(gtFile)

max_t = 1000
printed_particles = 10

trajectories = []
velocities = []
positions = []

gtTrajectory = []
gtVelocity = []
gtPosition = []

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


def main():
    readTrajectories()
    readGroundTruth()
    runSimulation()

    for i in range(0, min(printed_particles, len(trajectories))):
        traj = trajectories[i]
        plt.plot(traj[:min(len(traj), max_t)], alpha=0.5, linestyle="dashed") 
    plt.plot(gtTrajectory[:min(len(gtTrajectory), max_t)], alpha=1.0)
        
    plt.xlabel('time') 
    plt.ylabel('acceleration') 
    plt.title('acceleration-time') 
        
    plt.show()
    plt.savefig('plots/accel.png')

    plt.clf()



    for i in range(0, min(printed_particles, len(trajectories))):
        vel = velocities[i]
        plt.plot(vel[:min(len(traj), max_t)], alpha=0.5, linestyle="dashed") 
    plt.plot(gtVelocity[:min(len(gtTrajectory), max_t)], alpha=1.0)
        
    plt.xlabel('time') 
    plt.ylabel('velocity') 
    plt.title('velocity-time') 
        
    plt.show()
    plt.savefig('plots/velocity.png')

    plt.clf()

    
    
    for i in range(0, min(printed_particles, len(trajectories))):
        pos = positions[i]
        plt.plot(pos[:min(len(traj), max_t)], alpha=0.5, linestyle="dashed") 
    plt.plot(gtPosition[:min(len(gtTrajectory), max_t)], alpha=1.0)
        
    plt.xlabel('time') 
    plt.ylabel('position') 
    plt.title('position-time') 
    
    plt.show()
    plt.savefig('plots/position.png')
    return

if __name__ == "__main__":
    main()
