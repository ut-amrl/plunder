#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>

#include "pf.cpp"

using namespace std;

#define PF_SEED time(0)

/*
 * Configuration: Default parameters
 */

static Robot r (6, -5, 15, 100, normal_distribution<double>(0.0, 3.0), 0.9, 0);
static int N = 100;
static double resampleThreshold = 0.5;
static double LAStddev = 10.0;

// Global variables
static const char* inputFile = "../accSim/data.csv";
static const char* outputFile = "pf.csv";
static const int maxTimeSteps = 1000;

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
    for(int i = 0; i < maxTimeSteps; i++){
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
void writeData(const char* file, vector<vector<HA>> trajectories){
    ofstream outFile;
    outFile.open(file);

    for(vector<HA> traj : trajectories){
        for(int i = 0; i < traj.size(); i++){
            if(traj[i] == ACC){
                outFile << r.accMax;
            } else if (traj[i] == DEC){
                outFile << r.decMax;
            } else {
                outFile << 0;
            }
            if(i != traj.size() - 1){
                outFile << ",";
            }
        }
        outFile << endl;
    }


    outFile.close();
}

// ---------------------------------------------------------------------------------------------------------------------

vector<vector<HA>> runPF(vector<Obs> dataObs, vector<LA> dataLa){
    
    MarkovSystem<HA, LA, Obs, Robot> ms (&sampleInitialHA, &ASP, &logLikelihoodGivenMotorModel, r);

    PF<HA, LA, Obs, Robot> pf (&ms, dataObs, dataLa);
    pf.forward_filter(N, resampleThreshold);
    
    vector<vector<HA>> trajectories = pf.retrieveTrajectories();
    // for(vector<HA> traj : trajectories){
    //     for(int i = 0; i < traj.size(); i++){
    //         if(traj[i] == ACC){
    //             cout << r.accMax;
    //         } else if (traj[i] == DEC){
    //             cout << r.decMax;
    //         } else {
    //             cout << 0;
    //         }
    //         if(i != traj.size() - 1){
    //             cout << ",";
    //         }
    //     }
    //     cout << endl;
    // }

    return trajectories;
}


int main(int argc, char** argv){

    // Reading parameters
    if(argc > 1){
        if(argc < 12){
            cout << "Please run in the following manner: ./gen <robot test set> <model> <mean error> <error standard deviation> <high-level error>" << endl;
            exit(0);
        }

        r = Robot(stod(argv[1]), stod(argv[2]), stod(argv[3]), stod(argv[4]), normal_distribution<double>(stod(argv[5]), stod(argv[6])), stod(argv[7]), stod(argv[8]));
        N = stoi(argv[9]);
        resampleThreshold = stod(argv[10]);
        LAStddev = stod(argv[11]);
    }

    // Initialization
    srand(PF_SEED);

    vector<Obs> dataObs;
    vector<LA> dataLa;
    readData(inputFile, dataObs, dataLa);
    
    vector<vector<HA>> trajectories = runPF(dataObs, dataLa);

    writeData(outputFile, trajectories);
}