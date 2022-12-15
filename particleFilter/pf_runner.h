#pragma once

#include "pf.h"
#include "accSim/robotSets.h"
#include "settings.h"

using namespace std;

// ----- Markov System Parameters ---------------------------------------------

// Initial distribution
HA sampleInitialHA(){
    // Uniformly Randomly Distributed
    int mod = rand() % numHA;
    return to_label(mod);
}

double pfMotorModel(State state, Robot& r, LA nextLA) {
    double mean = 0;
    double stddev = min(r.accMax, abs(r.decMax));
    if(state.ha == ACC) mean = r.accMax;
    if(state.ha == DEC) mean = r.decMax;

    return logpdf(nextLA.acc, mean, obsLikelihoodStrength * stddev);
}

double robotMotorModel(State state, Robot& r, LA nextLA) {
    double mean = motorModel(state, r, false).acc; // should be using the previous LA
    return logpdf(nextLA.acc, mean, obsLikelihoodStrength * stddevError);
}


// Calculate probability of observing given LA with a hypothesized high-level action, then take natural log
double logLikelihoodGivenMotorModel(State state, Robot& r, LA nextLA){
    if(useSimplifiedMotorModel){
        return pfMotorModel(state, r, nextLA);
    }
    return robotMotorModel(state, r, nextLA);
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
double runFilter(vector<vector<HA>>& trajectories, int N, int M, double resampleThreshold, Robot& r, vector<Obs>& dataObs, vector<LA>& dataLa, asp* asp){

    // Initialization
    srand(0);
    MarkovSystem<HA, LA, Obs, Robot> ms (&sampleInitialHA, asp, &logLikelihoodGivenMotorModel, r);
    ParticleFilter<HA, LA, Obs, Robot> pf (&ms, dataObs, dataLa);
    resampCount = 0;
    
    // Run particle filter
    double obs_likelihood = pf.forwardFilter(N, resampleThreshold);

    pf.retrieveTrajectories(trajectories, M);

    // cout << "resample count: " << resampCount << endl;
    if(useSmoothTrajectories){
        trajectories = pf.smoothTrajectories(trajectories);
    }

    return obs_likelihood;
}

// Read input, run filter, write output
double filterFromFile(vector<vector<HA>>& trajectories, int N, int M, double resampleThreshold, Robot& r, string inputFile, string outputFile, vector<Obs>& dataObs, vector<LA>& dataLa, asp* asp){
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