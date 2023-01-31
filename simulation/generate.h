#pragma once

#include "robotSets.h"
#include "robot.h"

using nlohmann::json;
using namespace std;
using namespace AST;
using namespace SETTINGS;

// ----- Simulation ------------------------------------------------

string getCsvTitles(){
    string s = "time, ";
    for(Var each: Obs_vars){
        if(count(LA_vars.begin(), LA_vars.end(), each.name_) > 0){
            s += "LA.";
        }
        s += each.name_ + ", ";
    }
    s += "HA";
    return s;
}

string getCsvRow(State state, double t){
    string s = to_string(t) + ", ";
    for(Var each: Obs_vars){
        s += to_string(state.get(each.name_)) + ", ";
    }
    s += std::to_string(state.ha);
    return s;
}

vector<Trajectory> gen_trajectories(asp* asp, double gen_accuracy) {

    cout << "--------------Simulation---------------" << endl;
    cout << "Using " << VALIDATION_SET << " robots" << endl;

    // Initialization
    vector<State> robots = getInitStates();
    
    vector<Trajectory> trajectories;
    for(uint i = 0; i < VALIDATION_SET; i++){

        Robot r (robots[i]);
        Trajectory traj;
        
        // Run simulation
        for(double t = 0; t < T_TOT; t += T_STEP){
            r.updateObs(physicsModel);
            r.runASP(asp);
            HA prev_ha = (t == 0) ? 0 : traj.get(t-1).ha;
            r.state.ha = pointError(prev_ha, r.state.ha, gen_accuracy);
            r.updateLA(motorModel);

            traj.append(r.state);
        }

        trajectories.push_back(traj);
    }

    return trajectories;
}

void print_traj(Trajectory& traj) {
    cout << "Printing trajectory with total time " << traj.size() * T_STEP << "...\n";
    for (int i = 1; i < traj.size(); i++) {
        if(traj.get(i).ha != traj.get(i-1).ha){
            cout << print(traj.get(i-1).ha) << " --> " << print(traj.get(i).ha) << " at time " << i * T_STEP << endl;
        }
    }
    cout << endl;
}

void print_traj(vector<Trajectory>& trajectories) {
    for(Trajectory traj : trajectories) {
        print_traj(traj);
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

double execute_pure(Trajectory& traj, asp* asp){
    double log_obs = 0;

    for(int t = 0; t < traj.size(); t++){
        State last = (t == 0) ? State () : traj.get(t-1);
        State cur = traj.get(t);
        for(string each: LA_vars) {
            cur.put(each, last.get(each));
        }

        traj.set(t, asp(State(last.ha, cur.obs)));
        cur.ha = traj.get(t).ha;

        log_obs += obs_likelihood_given_model(cur, traj.get(t).obs);
    }

    return log_obs;
}
