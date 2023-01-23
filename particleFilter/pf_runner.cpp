#include "pf_runner.h"

using namespace std;
using namespace SETTINGS;

// ----- Main ---------------------------------------------

int main(){
    for(int i = 0; i < TRAINING_SET; i++){
        string in = SIM_DATA + to_string(i) + ".csv";
        string out = PF_TRAJ + to_string(i) + ".csv";

        Trajectory traj;
        vector<vector<HA>> trajectories;
        filterFromFile(trajectories, NUM_PARTICLES, max(SAMPLE_SIZE, PARTICLES_PLOTTED), in, out, traj, ASP_model);
        cout << "*";
    }
    cout << "\r";
    
    return 0;
}
