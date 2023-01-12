#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "steer", normal_distribution<double>(0.0, 0.1) },
    { "acc", normal_distribution<double>(0.0, 0.5) }
};

// I have no idea why these don't use the same units as the simulation??
// might create as variables
const double MAX_VEL = .375;
const double MIN_VEL = .225;
const double KP_A = 4;
const double STEER_ANGLE = 0.3;
const double STRAIGHTEN_ANGLE = 0.05;

// https://highway-env.readthedocs.io/en/latest/_modules/highway_env/vehicle/controller.html#ControlledVehicle.speed_control
Obs motorModel(State state, bool error){
    HA ha = state.ha;
    double acc_command = KP_A*(MAX_VEL-state.get("vx"));
    double dec_command = KP_A*(MIN_VEL-state.get("vx"));
    
    // Acceleration
    double acc = state.get("acc");
    if(ha == FASTER) {
        state.put("acc", acc_command);
    } else if (ha == SLOWER) {
        state.put("acc", dec_command);
    } else {
        // Maintain last action (FASTER or SLOWER)
        if(acc < 0) state.put("acc", dec_command);
        else state.put("acc", acc_command);
    }

    // Steering
    // More complicated than this, but this (very crude) approximation does not require knowledge of the target lane
    double steer = state.get("steer");
    if (ha == LANE_LEFT) {
        state.put("steer", -STEER_ANGLE);
    } else if (ha == LANE_RIGHT) {
        state.put("steer", STEER_ANGLE);
    } else {
        if(abs(steer) > STEER_ANGLE - 3 * STRAIGHTEN_ANGLE) {
            state.put("steer", ((steer > 0) ? -1 : 1) * STRAIGHTEN_ANGLE);
        } else {
            state.put("steer", steer / 2);
        }
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
