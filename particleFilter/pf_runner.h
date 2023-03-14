#pragma once

#include "pf.h"
#include "robotSets.h"
#include "settings.h"

using namespace std;
using namespace SETTINGS;

// ----- Markov System Parameters ---------------------------------------------

// Initial distribution
HA sampleInitialHA(){
    // Use default label
    return 0;
}

// ----- I/O ---------------------------------------------

// Read low-level action sequence and observed state sequence from file
void readData(string file, Trajectory& traj){
    ifstream infile;
    infile.open(file);
    string res;

    // header line
    getline(infile, res);
    istringstream iss_header (res);
    string comma;

    map<string, int> inds;
    string cur_var;
    int count = 0;
    while(getline(iss_header, cur_var, ',')) {
        trim(cur_var);
        if(cur_var.substr(0, 3) == "LA.")
            cur_var = cur_var.substr(3);

        inds[cur_var] = count;
        count++;
    }


    // data lines
    while(getline(infile, res)){
        istringstream iss (res);

        vector<double> vals;
        for(int i = 0; i < count; i++){
            double d; iss >> d >> comma;
            vals.push_back(d);
        }

        State state;
        for(Var each: Obs_vars) {
            state.put(each.name_, vals[inds[each.name_]]);
        }

        if(inds.count("HA") > 0) {
            state.set_ha(vals[inds["HA"]]);
        }

        traj.append(state);
    }
}

// Write high-level action sequences (trajectories) to file
void writeData(string file, vector<vector<HA>>& trajectories){
    ofstream outFile;
    outFile.open(file);

    for(vector<HA>& traj : trajectories){
        for(uint i = 0; i < traj.size(); i++){
            outFile << traj[i];
            if(i != traj.size() - 1){
                outFile << ",";
            }
        }
        outFile << endl;
    }

    outFile.close();
}


// ----- Particle Filter ---------------------------------------------

// Full trajectory generation with particle filter
double runFilter(vector<vector<HA>>& trajectories, int N, int M, Trajectory& traj, asp* asp){

    // Initialization
    srand(0);
    ParticleFilter pf (traj, asp, &sampleInitialHA, &obs_likelihood_given_model);
    resampCount = 0;
    
    // Run particle filter
    double obs_likelihood = pf.forwardFilter(N);

    pf.retrieveTrajectories(trajectories, M);

    if(DEBUG)
        cout << "resample count: " << resampCount << endl;

    return obs_likelihood;
}

// Read input, run filter, write output
double filterFromFile(vector<vector<HA>>& trajectories, int N, int M, string inputFile, string outputFile, Trajectory traj, asp* asp){
    // Read input
    if(traj.size() == 0){
        readData(inputFile, traj);
    }

    double obs_likelihood = runFilter(trajectories, N, M, traj, asp);

    // Write results
    writeData(outputFile, trajectories);

    return obs_likelihood;
}