#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// TODO
// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "steer", normal_distribution<double>(0.0, 0.1) },
    { "acc", normal_distribution<double>(0.0, 0.1) }
};

Obs motorModel(State state, bool error){
    return state.obs;
}


HA ASP_model(State state){
    HA ha;
    bool front_clear = flip(logistic(-50, -0.15, state.get("f_x")));
    bool left_clear = flip(logistic(-50, -0.15, state.get("l_x")));
    bool right_clear = flip(logistic(-50, -0.15, state.get("r_x")));

    if(front_clear) ha=FASTER;
    else if(left_clear) ha=LANE_LEFT;
    else if(right_clear) ha=LANE_RIGHT;
    else ha=SLOWER;

    return ha;
}
Obs physicsModel(State state, double t_step){}
