#pragma once

#include "robotSets.h"
#include "robot.h"
#include "ast/ast.hpp"

using nlohmann::json;
using namespace std;
using namespace AST;
using namespace SETTINGS;

// ----- Simulation ------------------------------------------------

string getCsvTitles(){
    return "time, x, v, LA, HA";
}

string getCsvRow(State state, double t){
    return to_string(t) + ", " + to_string(state.obs.pos) + ", " + to_string(state.obs.vel)
            + ", " + to_string(state.la.acc) + ", " + print(state.ha);
}

vector<Trajectory> gen_trajectories(int robot_set, asp* asp, double gen_accuracy) {

    cout << "--------------Simulation---------------" << endl;
    cout << "Running 1-D kinematic car simulation:\n";
    cout << "Using " << NUM_ROBOTS << " robots and low-level action standard deviation=" << STDDEV_ERROR << endl;

    // Initialization
    vector<Robot> robots = getRobotSet(robot_set);
    
    vector<Trajectory> trajectories;

    for(uint i = 0; i < NUM_ROBOTS; i++){
        Trajectory traj(robots[i]);

        // Run simulation
        for(double t = 0; t < T_TOT; t += T_STEP){

            robots[i].updateObs();
            robots[i].runASP(asp);
            robots[i].state.ha = pointError(robots[i].state.ha, gen_accuracy);
            robots[i].updateLA();

            traj.append(robots[i].state);
        }

        trajectories.push_back(traj);
    }

    return trajectories;
}

void print_traj(Trajectory& traj) {
    cout << "Printing trajectory with total time " << traj.size() * T_STEP << "...";
    for (int i = 1; i < traj.size(); i++) {
        if(traj.get(i).ha != traj.get(i-1).ha){
            cout << print(traj.get(i-1).ha) << " --> " << print(traj.get(i).ha) << " at time " << i * T_STEP << "\n";
        }
    }
    cout << "\n";
}

void print_traj(vector<Trajectory>& trajectories) {
    for(Trajectory traj : trajectories) {
        print_traj(trajectories);
    }
}

void write_traj(vector<Trajectory>& traj, string outputPath){
    cout << "Printing trajectories to " << outputPath << "\n\n\n";

    // Generate csv files
    for(uint i = 0; i < traj.size(); i++){

        // Setup CSV file
        ofstream csvFile;
        // cout << "Filling CSV with simulation data: " << outputPath << to_string(i) << ".csv" << endl;
        csvFile << fixed << setprecision(PRECISION);
        csvFile.open(outputPath + to_string(i) + ".csv");
        csvFile << getCsvTitles() << "\n";
        
        for(double t = 0; t < traj[i].size(); t++) {
            // Print trace
            csvFile << getCsvRow(traj[i].get(t), t) << "\n";
        }

        csvFile.close();
    }
}

void execute_pure(Trajectory& traj, asp* asp){

    // Turn off point error
    bool point_error = USE_POINT_ERROR;
    USE_POINT_ERROR = false;

    for(uint32_t t = 0; t < traj.size(); t++){
        State last = (t == 0) ? State {} : traj.get(t-1);
        State cur = traj.get(t);
        State s { last.ha, last.la, cur.obs };

        traj.traj[t].ha = asp(s, traj.r);
    }

    // Restore point error
    USE_POINT_ERROR = point_error;
}
