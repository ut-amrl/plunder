#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// TODO
// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "steer", normal_distribution<double>(0.0, 0.01) },
    { "acc", normal_distribution<double>(0.0, 0.01) }
};

// might create as variables
double MAX_VEL = .375;
double KP_A = 1.6666666666666667;


// https://highway-env.readthedocs.io/en/latest/_modules/highway_env/vehicle/controller.html#ControlledVehicle.speed_control
// state contains the current Obs and current LA but future HA
// returns future Obs
Obs motorModel(State state, bool error){
    HA ha = state.ha;
    
    if(ha == FASTER) {
        state.put("acc", KP_A*(MAX_VEL-state.get("vx")));
    } else if (ha == SLOWER) {
        state.put("acc", KP_A*(0-state.get("vx")));
    } else if (ha == LANE_LEFT) {
        if(state.get("acc")<0) state.put("acc", KP_A*(0-state.get("vx")));
        else state.put("acc", KP_A*(MAX_VEL-state.get("vx")));
        // more complicated than this, might implement later
        state.put("steer", .03);
    } else if (ha == LANE_RIGHT) {
        if(state.get("acc")<0) state.put("acc", KP_A*(0-state.get("vx")));
        else state.put("acc", KP_A*(MAX_VEL-state.get("vx")));
        // more complicated than this, might implement later
        state.put("steer", -.03);
    }

    return state.obs;
}


HA ASP_model(State state){
    HA ha;

    bool front_clear = flip(logistic(0.15, 75, state.get("f_x")));
    bool left_clear = flip(logistic(0.15, 50, state.get("l_x")));
    bool right_clear = flip(logistic(0.15, 50, state.get("r_x")));

    if(front_clear) ha=FASTER; // No car in front: accelerate
    else if(left_clear) ha=LANE_LEFT; // Merge left
    else if(right_clear) ha=LANE_RIGHT; // Merge right
    else ha=SLOWER; // Nowhere to go: slow down

    return ha;
}

Obs physicsModel(State state, double t_step){}
