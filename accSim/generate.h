#pragma once

#include "robotSets.h"
#include "robot.h"
#include "ast/ast.hpp"

using nlohmann::json;
using namespace std;
using namespace AST;

// ----- Simulation ------------------------------------------------

string getCsvTitles(){
    return "time, x, v, LA, HA";
}

string getCsvRow(State state, double t){
    return to_string(t) + ", " + to_string(state.obs.pos) + ", " + to_string(state.obs.vel)
            + ", " + to_string(state.la.acc) + ", " + to_string(state.ha);
}

vector<Trajectory> gen_trajectories(int robot_set, int use_model, double gen_accuracy) {

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
            robots[i].runASP(ASP_model(use_model));
            robots[i].state.ha = pointError(robots[i].state.ha, gen_accuracy);
            robots[i].updateLA();

            traj.append(robots[i].state);
        }

        trajectories.push_back(traj);
    }

    return trajectories;
}

void print_traj(vector<Trajectory> trajectories) {
    for(Trajectory traj : trajectories) {
        for (int i = 1; i < traj.T; i++) {
            if(traj.get(i).ha != traj.get(i-1).ha){
                cout << to_string(traj.get(i-1).ha) << " --> " << to_string(traj.get(i).ha) << " at time " << i * T_STEP << "\n";
            }
        }
        cout << "\n";
    }
}

void write_traj(vector<Trajectory> traj, string outputPath){
    cout << "Printing trajectories to " << outputPath << "\n\n\n";

    // Generate csv files
    for(uint i = 0; i < traj.size(); i++){

        // Setup CSV file
        ofstream csvFile;
        // cout << "Filling CSV with simulation data: " << outputPath << to_string(i) << ".csv" << endl;
        csvFile << fixed << setprecision(PRECISION);
        csvFile.open(outputPath + to_string(i) + ".csv");
        csvFile << getCsvTitles() << "\n";
        
        for(double t = 0; t < traj[i].T; t++) {
            // Print trace
            csvFile << getCsvRow(traj[i].get(t), t) << "\n";
        }

        csvFile.close();
    }
}

void executeASP(Robot& r, string outputFile, vector<Obs>& dataObs, asp* asp){

    ofstream outFile;
    outFile.open(outputFile);

    bool pointError = USE_POINT_ERROR;
    USE_POINT_ERROR = false;

    for(uint n=0; n<PARTICLES_PLOTTED; n++){
        r.reset();
        outFile << r.state.ha << ",";
        for(uint t=1; t<dataObs.size(); t++){
            r.state.obs = dataObs[t];
            r.runASP(asp);
            r.updateLA(false);
            outFile << r.state.ha;
            if(t!=dataObs.size()-1) outFile << ",";
        }
        outFile << endl;
    }
    outFile.close();

    USE_POINT_ERROR = pointError;
}
