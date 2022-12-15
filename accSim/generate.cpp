#include "generate.h"
#include "settings.h"

using namespace std;

// ----- Main ---------------------------------------------

int main(int argc, char** argv) {
    vector<Trajectory> traj = gen_trajectories(robotTestSet, model, genAccuracy);

    if(DEBUG)
        print_traj(traj);
    write_traj(traj, stateGenPath);
    
    return 0;
}