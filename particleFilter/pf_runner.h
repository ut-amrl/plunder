#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <string>
#include <cstring>

#include "pf.h"
#include "../accSim/asps.h"
#include "../accSim/robotSets.h"

using namespace std;

// ----- Markov System Parameters ---------------------------------------------

// Initial distribution
HA sampleInitialHA(){
    // Distribution 1: Point distribution
    // return ACC;

    // Distribution 2: Uniformly Randomly Distributed
    double rv = ((double) rand()) / RAND_MAX;
    if(rv <= 0.33){
        return ACC;
    } else if (rv <= 0.67){
        return DEC;
    } else {
        return CON;
    }
}

// Calculate pdf of N(mu, sigma) at x, then take the natural log
FLOAT logpdf(FLOAT x, FLOAT mu, FLOAT sigma){
    return (-log(sigma)) - (0.5*log(2*M_PI)) - 0.5*pow((x - mu)/sigma, 2);
}

// Calculate probability of observing given LA with a hypothesized high-level action, then take natural log
FLOAT logLikelihoodGivenMotorModel(Robot* r, LA la, HA ha, Obs obs){
    double mean = r->motorModel(ha, obs, false).acc;
    double stddev = r->accErrDistr.stddev();
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
void writeData(string file, Robot* r, vector<vector<HA>>& trajectories){
    ofstream outFile;
    outFile.open(file);

    for(vector<HA> traj : trajectories){
        for(uint i = 0; i < traj.size(); i++){
            outFile << r->motorModel(traj[i], Obs {}, false).acc;
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
vector<vector<HA>> runFilter(int N, double resampleThreshold, Robot* r, vector<Obs> dataObs, vector<LA> dataLa, asp_t* asp){

    // Initialization
    srand(PF_SEED);
    MarkovSystem<HA, LA, Obs, Robot> ms (&sampleInitialHA, asp, &logLikelihoodGivenMotorModel, r);
    ParticleFilter<HA, LA, Obs, Robot> pf (&ms, dataObs, dataLa);
    resampCount = 0;
    
    // Run particle filter
    pf.forwardFilter(N, resampleThreshold);
    vector<vector<HA>> trajectories = pf.retrieveTrajectories();

    // DEBUG
    cout << "resample count: " << resampCount << endl;

    return trajectories;
}

void filterFromFile(int N, double resampleThreshold, Robot* r, string inputFile, string outputFile, asp_t* asp){

    // Read input
    vector<Obs> dataObs;
    vector<LA> dataLa;
    readData(inputFile, dataObs, dataLa);

    vector<vector<HA>> trajectories = runFilter(N, resampleThreshold, r, dataObs, dataLa, asp);

    // Write results
    writeData(outputFile, r, trajectories);
}

void processPath(int N, double resampleThreshold, vector<Robot>& robots, string inputPath, string outputPath, int numFiles, asp_t* asp){
    for(int i = 0; i < numFiles; i++){
        string in = inputPath + to_string(i) + ".csv";
        string out = outputPath + to_string(i) + ".csv";

        filterFromFile(N, resampleThreshold, &robots[i], in, out, asp);
    }
}


