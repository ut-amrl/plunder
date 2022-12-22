#include "generate.h"
#include "settings.h"

using namespace std;
using namespace SETTINGS;

// ----- Main ---------------------------------------------

int main(int argc, char** argv) {
    vector<Trajectory> traj = gen_trajectories(ROBOT_SET, ASP_model(GT_ASP), GEN_ACCURACY);

    if(DEBUG)
        print_traj(traj);
    write_traj(traj, SIM_DATA);
    
    return 0;
}