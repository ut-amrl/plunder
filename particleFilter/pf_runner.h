#pragma once

#include "pf.h"
#include "robotSets.h"
#include "settings.h"

using namespace std;
using namespace SETTINGS;

// ----- Markov System Parameters ---------------------------------------------

// Initial distribution
HA sampleInitialHA(){
    // Uniformly Randomly Distributed
    return rand() % numHA;
}

// Calculate probability of observing given LA with a hypothesized high-level action, then take natural log
double obs_likelihood_given_model(State state, Obs nextLA){
    Obs mean = motorModel(state, false); // Use the previous state + current HA
    double obs_log = 0;
    for(string each: LA_vars) {
        obs_log += logpdf(nextLA.get(each), mean.get(each), OBS_LIKELIHOOD_STRENGTH * STDDEV_ERROR);
    }
    return obs_log;
}


// ----- I/O ---------------------------------------------

// Read low-level action sequence and observed state sequence from file
void readData(string file, Trajectory& traj){
    ifstream infile;
    infile.open(file);
    string res;

    // header line
    getline(infile, res);

    // data lines
    while(getline(infile, res)){
        istringstream iss (res);
        string comma;

        float time; iss >> time >> comma;
        State state;
        for(Var each: Obs_vars) {
            double d; iss >> d >> comma;
            state.put(each.name_, d);
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
    
    if(SMOOTH_TRAJECTORIES){
        trajectories = pf.smoothTrajectories(trajectories);
    }

    return obs_likelihood;
}

// Read input, run filter, write output
double filterFromFile(vector<vector<HA>>& trajectories, int N, int M, string inputFile, string outputFile, Trajectory& traj, asp* asp){
    // Read input
    if(traj.size() == 0){
        readData(inputFile, traj);
    }

    double obs_likelihood = runFilter(trajectories, N, M, traj, asp);

    // Write results
    writeData(outputFile, trajectories);

    return obs_likelihood;
}