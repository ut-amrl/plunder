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

    bool front_clear = flip(logistic(0.15, 50, state.get("f_x")));
    bool left_clear = flip(logistic(0.15, 50, state.get("l_x")));
    bool right_clear = flip(logistic(0.15, 50, state.get("r_x")));

    if(front_clear) ha=FASTER; // No car in front: accelerate
    else if(left_clear) ha=LANE_LEFT; // Merge left
    else if(right_clear) ha=LANE_RIGHT; // Merge right
    else ha=SLOWER; // Nowhere to go: slow down

    return ha;
}

Obs physicsModel(State state, double t_step){}
