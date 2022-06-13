#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>

#include "pf.cpp"

using namespace std;

#define PF_SEED time(0)

static const char* inputFile = "../accSim/data.csv";
static const char* outputFile = "pf.csv";

static const double LAStddev = 10.0;
static const int maxDataSteps = 1000;

static const Robot r (6, -5, 15, 150, normal_distribution<double>(0.0, 0.1), 1, 0);
static const int N = 100;
static const double resampleThreshold = 0.5;

// ---------------------------------------------------------------------------------------------------------------------
// Markov System Configuration

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
HA ASP(Robot r, HA prevHA, Obs prevObs){
    r.x = prevObs.pos;
    r.v = prevObs.vel;
    r.ha = prevHA;

    r.changeHA();
    
    return r.ha;
}

// Calculate pdf of N(mu, sigma) at x, then take the natural log
FLOAT logpdf(FLOAT x, FLOAT mu, FLOAT sigma){
    return (-log(sigma)) - (0.5*log(2*M_PI)) - 0.5*pow((x - mu)/sigma, 2);
}

// Calculate probability of observing given LA with a hypothesized high-level action, then take natural log
FLOAT logLikelihoodGivenMotorModel(Robot r, LA la, HA ha, Obs obs){
    double laMean = (ha == ACC) ? r.accMax : (ha == DEC) ? r.decMax : 0;
    double res = logpdf(la.acc, laMean, LAStddev);
    return res;
}



// ---------------------------------------------------------------------------------------------------------------------

// Read low-level action sequence and observed state sequence from file
void readData(const char* file, vector<Obs>& dataObs, vector<LA>& dataLA){

    ifstream infile;
    infile.open(file);
    string res;
    // header line
    getline(infile, res);
    // data lines
    for(int i = 0; i < maxDataSteps; i++){
        if(!getline(infile, res))
            break;
        istringstream iss (res);
        float time; string comma1;
        float x; string comma2;
        float v; string comma3;
        float a;
        iss >> time >> comma1 >> x >> comma2 >> v >> comma3 >> a;

        Obs obs;
        obs.pos = x;
        obs.vel = v;
        dataObs.push_back(obs);

        LA la;
        la.acc = a;
        dataLA.push_back(la);
    }
}

// Write high-level action sequences (trajectories) to file


// ---------------------------------------------------------------------------------------------------------------------

vector<vector<HA>> runPF(vector<Obs> dataObs, vector<LA> dataLa){
    
    MarkovSystem<HA, LA, Obs, Robot> ms (&sampleInitialHA, &ASP, &logLikelihoodGivenMotorModel, r);
    

    PF<HA, LA, Obs, Robot> pf (&ms, dataObs, dataLa);
    pf.forward_filter(N, resampleThreshold);
    
    vector<vector<HA>> traj = pf.retrieveTrajectories();
    cout << "PF test - trajectories: " << endl;
    for(vector<HA> each: traj){
        for(HA ha: each){
            cout << ha << " ";
        }
        cout << endl;
    }
    cout << endl;

    return traj;
}


int main(){
    srand(PF_SEED);

    vector<Obs> dataObs;
    vector<LA> dataLa;
    readData(inputFile, dataObs, dataLa);
    
    vector<vector<HA>> trajectories = runPF(dataObs, dataLa);
}