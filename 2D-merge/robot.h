#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "steer", normal_distribution<double>(0.0, 0.03) },
};

double KP_H = 0.3;
double TURN_HEADING = 0.2;
double TURN_TARGET = 30;

double lane_diff = 4;

double laneFinder(double y) {
    return round(y / lane_diff);
}

// Motor model matches merge.py
Obs motorModel(State state, bool error){
    HA ha = state.ha;

    double target_heading = 0.0;
    
    if(ha == IDLE) {
        // Follow current lane
        double target_y = laneFinder(state.get("y")) * lane_diff;
        target_heading = atan((target_y - state.get("y")) / TURN_TARGET);
    } else if (ha == LANE_LEFT) {
        // Attain leftmost heading
        target_heading = -TURN_HEADING;
    } else if (ha == LANE_RIGHT) {
        // Attain rightmost heading
        target_heading = TURN_HEADING;
    }

    double steer = (target_heading - state.get("heading")) * KP_H;
    state.put("steer", steer);

    return state.obs;
}

HA ASP_model(State state){
    HA ha;

    bool in_left_lane = flip(logistic(2, -10, state.get("y")));
    bool right_clear = flip(logistic(45, 1, state.get("r_x")));

    if(in_left_lane) {
        if(right_clear) {
            ha = LANE_RIGHT;
        } else {
            ha = IDLE;
        }
    } else {
        if(right_clear) {
            ha = IDLE;
        } else {
            ha = LANE_LEFT;
        }
    }

    return ha;
}

Obs physicsModel(State state, double t_step){}
