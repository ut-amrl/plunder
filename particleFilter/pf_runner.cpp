#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <string>
#include <cstring>

#include "pf.cpp"
#include "../accSim/asp_ex.cpp"
#include "../accSim/robotSets.h"

using namespace std;

#define PF_SEED time(0)

// ----- Default Configuration ---------------------------------------------

static vector<Robot> robots = { Robot(5, -4, 12, 100, normal_distribution<double>(0.0, 1.0), 0.8) };
static int N = 1000;
static double resampleThreshold = 0.1;

// File paths
static string inputPath = "accSim/out/data";
static string outputPath = "particleFilter/out/pf";

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

// Run robot-based action-selection policy
HA ASP(HA ha, Obs obs, Robot* r){
    return ASP_model(ha, obs, r);
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

void runFilter(int N, double resampleThreshold, Robot* r, string inputFile, string outputFile){

    // Read input
    vector<Obs> dataObs;
    vector<LA> dataLa;
    readData(inputFile, dataObs, dataLa);

    // Initialization
    srand(PF_SEED);
    MarkovSystem<HA, LA, Obs, Robot> ms (&sampleInitialHA, &ASP, &logLikelihoodGivenMotorModel, r);
    ParticleFilter<HA, LA, Obs, Robot> pf (&ms, dataObs, dataLa);
        
    // Run particle filter
    pf.forwardFilter(N, resampleThreshold);
    vector<vector<HA>> trajectories = pf.retrieveTrajectories();

    // Write results
    writeData(outputFile, r, trajectories);
}

void processPath(int N, double resampleThreshold, vector<Robot>& robots, string inputPath, string outputPath, int numFiles){
    for(int i = 0; i < numFiles; i++){
        string in = inputPath + to_string(i) + ".csv";
        string out = outputPath + to_string(i) + ".csv";

        runFilter(N, resampleThreshold, &robots[i], in, out);
    }
}


// ----- Main ---------------------------------------------

int main(int argc, char** argv){
    // Reading parameters
    if(argc > 1){
        if(argc < 8){
            cout << "Please run in the following manner: ./pf <robot test set> <mean error> <error std dev> <high-level success rate> <model> <numParticles> <resampleThreshold>" << endl;
            exit(0);
        }

        vector<Robot> robots = getRobotSet(stoi(argv[1]), normal_distribution<double>(stod(argv[2]), stod(argv[3])), stod(argv[4]));
        setModel(stod(argv[5]));

        processPath(stoi(argv[6]), stod(argv[7]), robots, inputPath, outputPath, robots.size());
    }
}