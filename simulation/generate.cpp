#include "generate.h"
#include "settings.h"

using namespace std;
using namespace SETTINGS;

// ----- Main ---------------------------------------------

int main() {
    vector<Trajectory> traj = gen_trajectories(ASP_model, GEN_ACCURACY);

    if(DEBUG)
        print_traj(traj);
    write_traj(traj, SIM_DATA);
    
    return 0;
}