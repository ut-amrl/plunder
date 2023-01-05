#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// TODO
// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "steer", normal_distribution<double>(0.0, 0.05) },
    { "acc", normal_distribution<double>(0.0, 1) }
};

// I have no idea why these don't use the same units as the simulation??
// might create as variables
double MAX_VEL = .375;
double MIN_VEL = .225;
double KP_A = 4;
double STEER_ANGLE = 0.3;


// https://highway-env.readthedocs.io/en/latest/_modules/highway_env/vehicle/controller.html#ControlledVehicle.speed_control
Obs motorModel(State state, bool error){
    HA ha = state.ha;
    
    if(ha == FASTER) {
        state.put("acc", KP_A*(MAX_VEL-state.get("vx")));
    } else if (ha == SLOWER) {
        state.put("acc", KP_A*(MIN_VEL-state.get("vx")));
    } else if (ha == LANE_LEFT) {
        // Maintain last action (FASTER or SLOWER)
        if(state.get("acc") < 0) state.put("acc", KP_A*(MIN_VEL-state.get("vx")));
        else state.put("acc", KP_A*(MAX_VEL-state.get("vx")));

        // more complicated than this, might implement later (left merge is followed by a slight right turn)
        state.put("steer", -STEER_ANGLE);
    } else if (ha == LANE_RIGHT) {
        // Maintain last action (FASTER or SLOWER)
        if(state.get("acc") < 0) state.put("acc", KP_A*(MIN_VEL-state.get("vx")));
        else state.put("acc", KP_A*(MAX_VEL-state.get("vx")));

        // more complicated than this, might implement later (right merge is followed by a slight left turn)
        state.put("steer", STEER_ANGLE);
    }

    return state.obs;
}


HA ASP_model(State state){
    HA ha;

    bool front_clear = flip(logistic(0.15, 90, state.get("f_x")));
    bool left_clear = flip(logistic(0.15, 90, state.get("l_x")));
    bool right_clear = flip(logistic(0.15, 90, state.get("r_x")));

    if(front_clear) ha=FASTER; // No car in front: accelerate
    else if(left_clear) ha=LANE_LEFT; // Merge left
    else if(right_clear) ha=LANE_RIGHT; // Merge right
    else ha=SLOWER; // Nowhere to go: slow down

    return ha;
}

Obs physicsModel(State state, double t_step){}
