#include "pf_runner.h"

using namespace std;

// ----- Main ---------------------------------------------

int main(int argc, char** argv){
    vector<Robot> robots = getRobotSet(robotTestSet);

    for(int i = 0; i < numRobots; i++){
        string in = stateGenPath + to_string(i) + ".csv";
        string out = trajGenPath + to_string(i) + ".csv";

        vector<Obs> dataObs;
        vector<LA> dataLa;
        vector<vector<HA>> trajectories;
        filterFromFile(trajectories, numParticles, numTrajectories, resampleThreshold, robots[i], in, out, dataObs, dataLa, ASP_model(model));
        cout << "*";
    }
    cout << "\r";
    
    return 0;
}
