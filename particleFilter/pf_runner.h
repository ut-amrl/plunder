#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <string>
#include <cstring>

#include "pf.h"
#include "accSim/asps.h"
#include "accSim/robotSets.h"
#include "settings.h"

using namespace std;

// ----- Markov System Parameters ---------------------------------------------

// Initial distribution
HA sampleInitialHA(){
    // Distribution 1: Point distribution
    return ACC;

    // Distribution 2: Uniformly Randomly Distributed
    // double rv = ((double) rand()) / RAND_MAX;
    // if(rv <= 0.33){
    //     return ACC;
    // } else if (rv <= 0.67){
    //     return DEC;
    // } else {
    //     return CON;
    // }
}

// Calculate pdf of N(mu, sigma) at x, then take the natural log
FLOAT logpdf(FLOAT x, FLOAT mu, FLOAT sigma){
    return (-log(sigma)) - (0.5*log(2*M_PI)) - 0.5*pow((x - mu)/sigma, 2);
}

// Calculate probability of observing given LA with a hypothesized high-level action, then take natural log
FLOAT logLikelihoodGivenMotorModel(Robot& r, LA la, HA ha, Obs obs, LA prevLA){
    double mean = r.motorModel(ha, obs, prevLA, false).acc; // should be using the previous LA
    double stddev = pf_stddevError;
    return logpdf(la.acc, mean, stddev);
}


// ----- I/O ---------------------------------------------

// Read low-level action sequence and observed state sequence from file
void readData(string file, vector<Obs>& dataObs, vector<LA>& dataLA){
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
        dataObs.push_back(obs);

        LA la = { .acc = a };
        dataLA.push_back(la);
    }
}

// Write high-level action sequences (trajectories) to file
void writeData(string file, Robot& r, vector<vector<HA>>& trajectories, vector<Obs>& dataObs){
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
double runFilter(vector<vector<HA>>& trajectories, int N, int M, double resampleThreshold, Robot& r, vector<Obs>& dataObs, vector<LA>& dataLa, asp_t* asp){

    // Initialization
    srand(PF_SEED);
    MarkovSystem<HA, LA, Obs, Robot> ms (&sampleInitialHA, asp, &logLikelihoodGivenMotorModel, r);
    ParticleFilter<HA, LA, Obs, Robot> pf (&ms, dataObs, dataLa);
    resampCount = 0;
    
    // Run particle filter
    double obs_likelihood = pf.forwardFilter(N, resampleThreshold);

    pf.retrieveTrajectories(trajectories, M);

    // cout << "resample count: " << resampCount << endl;

    return obs_likelihood;
}

// Read input, run filter, write output
double filterFromFile(vector<vector<HA>>& trajectories, int N, int M, double resampleThreshold, Robot& r, string inputFile, string outputFile, vector<Obs>& dataObs, vector<LA>& dataLa, asp_t* asp){
    // Read input
    if(dataObs.size() == 0 || dataLa.size() == 0){
        dataObs.clear(); dataLa.clear();
        readData(inputFile, dataObs, dataLa);
    }

    double obs_likelihood = runFilter(trajectories, N, M, resampleThreshold, r, dataObs, dataLa, asp);

    // Write results
    writeData(outputFile, r, trajectories, dataObs);

    return obs_likelihood;
}