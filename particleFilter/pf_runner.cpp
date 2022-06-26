#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <string>
#include <cstring>

#include "pf_runner.h"
#include "../settings.h"

using namespace std;

#define PF_SEED time(0)

// ----- Main ---------------------------------------------

int main(int argc, char** argv){
    vector<Robot> robots = getRobotSet(robotTestSet, normal_distribution<double>(meanError, stddevError), pointAccuracy);

    for(int i = 0; i < robots.size(); i++){
        string in = stateGenPath + to_string(i) + ".csv";
        string out = trajGenPath + to_string(i) + ".csv";

        vector<Obs> dataObs;
        vector<LA> dataLa;
        filterFromFile(numParticles, numTrajectories, resampleThreshold, robots[i], in, out, dataObs, dataLa, ASP_model(model));
    }
    
    return 0;
}
