#pragma once

#include "pf.h"
#include "accSim/robotSets.h"
#include "settings.h"

using namespace std;
using namespace SETTINGS;

// ----- Markov System Parameters ---------------------------------------------

// Initial distribution
HA sampleInitialHA(){
    // Uniformly Randomly Distributed
    int mod = rand() % numHA;
    return to_label(mod);
}

// Calculate probability of observing given LA with a hypothesized high-level action, then take natural log
double obs_likelihood_given_model(State state, Robot& r, LA nextLA){
    double mean = motorModel(state, r, false).acc; // should be using the previous LA
    return logpdf(nextLA.acc, mean, OBS_LIKELIHOOD_STRENGTH * STDDEV_ERROR);
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
        float time, x, v, a; string comma;
        iss >> time >> comma >> x >> comma >> v >> comma >> a;

        Obs obs = { .pos = x, .vel = v };
        LA la = { .acc = a };

        traj.append(State { HA{}, la, obs });
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
    if(traj.T == 0){
        readData(inputFile, traj);
    }

    double obs_likelihood = runFilter(trajectories, N, M, traj, asp);

    // Write results
    writeData(outputFile, trajectories);

    return obs_likelihood;
}