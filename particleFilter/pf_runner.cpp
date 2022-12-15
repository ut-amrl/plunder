#include "pf_runner.h"

using namespace std;

// ----- Main ---------------------------------------------

int main(int argc, char** argv){
    vector<Robot> robots = getRobotSet(ROBOT_SET);

    for(int i = 0; i < NUM_ROBOTS; i++){
        string in = SIM_DATA + to_string(i) + ".csv";
        string out = PF_TRAJ + to_string(i) + ".csv";

        vector<Obs> dataObs;
        vector<LA> dataLa;
        vector<vector<HA>> trajectories;
        filterFromFile(trajectories, NUM_PARTICLES, NUM_TRAJECTORIES, RESAMPLE_THRESHOLD, robots[i], in, out, dataObs, dataLa, ASP_model(GT_ASP));
        cout << "*";
    }
    cout << "\r";
    
    return 0;
}
