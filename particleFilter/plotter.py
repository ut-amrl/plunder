import matplotlib.pyplot as plt
import numpy as np 
import csv
import os
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
import time
plt.rcParams.update({'font.size': 14})

# Initialization
trajectories = []
gtTrajectory = []
gtLA = []

useGT = True

settings = {}

# Default colors
colors = ["#F08080", "#9FE2BF", "#6495ED", "#FFBF00", "#CCCCFF", "#40E0D0", "#DE3163", "#DFFF00", "#34495E", "#FF7F50"]

# ----- I/O ---------------------------------------------=

# Reads in csv files (int)
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

def readGroundTruth(gtPath):
    global useGT

    while not os.path.exists(gtPath):
        time.sleep(1)
    gtReader = csv.reader(open(gtPath, "r"))
    title_row = next(gtReader)

    ha_index = -1
    for i in range(len(title_row)):
        if title_row[i].strip() == "HA":
            ha_index = i

    if ha_index == -1:
        useGT = False
        return
    
    for info in gtReader:
        if len(info)>0:
            gtTrajectory.append(int(float(info[ha_index].strip())))

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

    for traj in trajectories:
        traj = traj[0:maxTime]
    gtTrajectory = gtTrajectory[0:maxTime]

    times = []
    for t in range(maxTime):
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
    for i in range(len(actions)):
        ax1.bar(times, actions[i], width=1, bottom=cum_sum, color=colors[i])
        cum_sum = np.add(cum_sum, actions[i])

    # TODO: color map is just using whatever order it wants. I can't figure out how to force it to use our desired order
    ax1b.imshow(np.array(color_graph), cmap=ListedColormap(colors[0:len(actions)]), origin="lower", vmin=0, aspect='auto', interpolation='none')

    plt.xlabel('Time (s)')
    ax1.set_ylabel('high-level\nactions')
    ax1b.set_ylabel('high-level\nactions')

    if useGT:
        for i in range(len(gt)):
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

# Plots low-level actions
def plotSingleLA(inF, outP, gtF, iter, robot):
    global trajectories, gtTrajectory, gtLA
    
    gtFile = gtF + str(robot) + ".csv"
    la_names = readLA(gtFile)
    outPath = outP + "LA-" + str(iter) + "-" + str(robot) + "-"

    fig, axs = plt.subplots(len(gtLA))
    fig.suptitle("Low-level actions")
    plt.xlabel('Time (s)')

    color_count = 0
    for name in range(len(la_names)):
        handles = []

        label_name = la_names[name]
        
        inFile = inF + str(iter) + "-" + str(robot) + "-" + la_names[name] + ".csv"
        trajectories = []
        readTrajectoriesFloat(inFile)

        PARTICLES_PLOTTED = min(int(settings["SAMPLE_SIZE"]), len(trajectories))    
        
        maxTime = min(int(settings["PLOT_TIME"]), len(gtLA[0]))
        for arr in gtLA:
            del arr[maxTime:]
        for arr in trajectories:
            del arr[maxTime:]

        times = []
        for t in range(maxTime):
            times.append(t)

        if len(gtLA) == 1:
            axs.set_ylabel(label_name)
            axs.plot(times, gtLA[0], color=colors[0], linewidth=1.5)
            handles.append(Line2D([0], [0], label='actual', color=colors[0]))
            handles.append(Line2D([0], [0], label='predicted', color=colors[1]))
            for p in range(PARTICLES_PLOTTED):
                axs.plot(times, trajectories[p], color=colors[1], alpha=(1/PARTICLES_PLOTTED), linewidth=2.5) 

            axs.grid(linestyle='dotted')  
            axs.legend(handles=handles, loc='upper right')
        else:
            axs[name].set_ylabel(label_name)
            axs[name].plot(times, gtLA[name], color=colors[0], linewidth=1.5)
            handles.append(Line2D([0], [0], label='actual', color=colors[0]))
            handles.append(Line2D([0], [0], label='predicted', color=colors[1]))
            color_count += 1

            for p in range(PARTICLES_PLOTTED):
                axs[name].plot(times, trajectories[p], color=colors[1], alpha=(1/PARTICLES_PLOTTED), linewidth=2.5)
            color_count += 1

            axs[name].grid(linestyle='dotted')
            axs[name].legend(handles=handles, loc='upper right')

            for tick in axs[name].xaxis.get_major_ticks():
                tick.label.set_fontsize(15)
            for tick in axs[name].yaxis.get_major_ticks():
                tick.label.set_fontsize(15)

    fig.tight_layout()
    plt.grid(linestyle='dotted')
    plt.show()
    if os.path.exists(outPath[:outPath.rfind('/')]):
        plt.savefig(outPath + "graph.png")
    else:
        raise Exception('Plots folder deleted... restarting plotter')

    plt.clf()
    plt.close('all')

# Processes input and passes to figureHandler
def plotSingle(inF, outP, gtF, title, iter, robot):
    if not title == 'Particle filter outputs':
        plotSingleLA(inF, outP, gtF, iter, robot)

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

    PARTICLES_PLOTTED = min(int(settings["SAMPLE_SIZE"]), len(trajectories))

    max_action = 0
    for t in range(maxTime):
        for i in range(len(trajectories)):
            max_action = max(max_action, trajectories[i][t])
        if useGT:
            max_action = max(max_action, gtTrajectory[t])
    
    actions = []
    gt = []
    for i in range(max_action + 1):
        actions.append([0] * maxTime)
        gt.append([0] * maxTime)
         
    for t in range(maxTime):
        for i in range(len(trajectories)):
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


def plotSingleTimestep(inF, outP, gtF, title, iter, numRobots):
    for robot in range(numRobots):
        plotSingle(inF, outP, gtF, title, iter, robot)

def plotLikelihoods(likelihoodDataFile, likelihoodPlotFile, title, ylabel):
    vals = []
    with open(likelihoodDataFile) as f:
        lines = f.readlines()
        gt = float(lines[0].strip())
        for line in lines[1:]:
            vals.append(float(line.strip()))

    vals = vals[1:]
    
    plt.suptitle(title)
    plt.xlabel('Iteration')
    plt.xticks(range(0,16,2))
    plt.ylabel(ylabel)
    plt.plot(vals, linewidth=2, markersize=4, label="synthesized programs")
    # plt.axhline(y = gt, color = 'green', label="ground truth")
    plt.legend(loc="lower right")
    plt.tight_layout()
    plt.show()
    plt.savefig(likelihoodPlotFile)
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
    trainingInFile = settings["TRAINING_TRAJ"]
    validationInFile = settings["VALIDATION_TRAJ"]
    gtFile = settings["SIM_DATA"]
    trainingOutPath = settings["PLOT_PATH"]+"training/"
    testingOutPath = settings["PLOT_PATH"]+"testing/"
    validationOutPath = settings["PLOT_PATH"]+"validation/"

    try:
        print("Plotting ground truth...")
        for robot in range(int(settings["VALIDATION_SET"])):
            plotSingle(validationInFile, validationOutPath, gtFile, 'Ground Truth Robots', 'gt', robot)

        graph1 =    {   'inF': trainingInFile,
                        'outP': trainingOutPath,
                        'gtF': gtFile,
                        'title': 'Particle filter outputs'
                    }
        graph2 =    {   'inF': validationInFile,
                        'outP': testingOutPath,
                        'gtF': gtFile,
                        'title': 'ASP Test Run'
                    }

        for iter in range(int(settings["NUM_ITER"])):
            print("Plotting training graphs, iteration " + str(iter))
            plotSingleTimestep(graph1['inF'], graph1['outP'], graph1['gtF'], graph1['title'], iter, int(settings["TRAINING_SET"]))
            print("Plotting testing ASP graphs, iteration " + str(iter))
            plotSingleTimestep(graph2['inF'], graph2['outP'], graph2['gtF'], graph2['title'], iter, int(settings["VALIDATION_SET"]))
            print("Plotting likelihoods, iteration " + str(iter))
            plotLikelihoods(settings["LOG_OBS_PATH"] + "-training.txt", settings["PLOT_PATH"]+"training-likelihoods.png", "Training Set Likelihoods", 'Log Obs. Likelihood')
            plotLikelihoods(settings["LOG_OBS_PATH"] + "-testing.txt", settings["PLOT_PATH"]+"testing-likelihoods.png", "Testing Set Likelihoods", 'Log Obs. Likelihood')
            plotLikelihoods(settings["LOG_OBS_PATH"] + "-valid.txt", settings["PLOT_PATH"]+"validation-likelihoods.png", "Validation Set Likelihoods", 'Log Obs. Likelihood')
            plotLikelihoods(settings["PCT_ACCURACY"] + "-training.txt", settings["PLOT_PATH"]+"training-accuracy.png", "Training Set Accuracy", 'Percent Accuracy')
            plotLikelihoods(settings["PCT_ACCURACY"] + "-testing.txt", settings["PLOT_PATH"]+"testing-accuracy.png", "Testing Set Accuracy", 'Percent Accuracy')
            plotLikelihoods(settings["PCT_ACCURACY"] + "-valid.txt", settings["PLOT_PATH"]+"validation-accuracy.png", "Validation Set Accuracy", 'Percent Accuracy')
        
        # for robot in range(int(settings["VALIDATION_SET"])):
        #     plotSingle(validationInFile, validationOutPath, gtFile, 'Final Outputs', str(int(settings["NUM_ITER"]) - 1), robot)
            
    except Exception as e:
        print(e)
        pass

if __name__ == "__main__":
    main()