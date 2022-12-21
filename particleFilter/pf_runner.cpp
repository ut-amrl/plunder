#include "pf_runner.h"

using namespace std;
using namespace SETTINGS;

// ----- Main ---------------------------------------------

int main(int argc, char** argv){
    vector<Robot> robots = getRobotSet(ROBOT_SET);

    for(int i = 0; i < NUM_ROBOTS; i++){
        string in = SIM_DATA + to_string(i) + ".csv";
        string out = PF_TRAJ + to_string(i) + ".csv";

        Trajectory traj (robots[i]);
        vector<vector<HA>> trajectories;
        filterFromFile(trajectories, NUM_PARTICLES, NUM_TRAJECTORIES, in, out, traj, ASP_model(GT_ASP));
        cout << "*";
    }
    cout << "\r";
    
    return 0;
}
