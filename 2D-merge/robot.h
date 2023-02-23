#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "steer", normal_distribution<double>(0.0, 0.01) },
    { "acc", normal_distribution<double>(0.0, 1) }
};

double TURN_HEADING = 0.15;
double TURN_TARGET = 30;

double min_velocity = 16;
double max_velocity = 30;
double lane_diff = 4;

double laneFinder(double y) {
    return round(y / lane_diff);
}

// Motor model matches highway2d.py
Obs motorModel(State state, bool error){
    HA ha = state.ha;

    double acc = 0.0;
    double target_heading = 0.0;
    
    if(ha == FASTER) {
        // Attain max speed
        acc = min(max_velocity - state.get("vx"), state.get("acc") + 0.5);

        // Follow current lane
        double target_y = laneFinder(state.get("y")) * lane_diff;
        target_heading = atan((target_y - state.get("y")) / TURN_TARGET);
    } else if (ha == SLOWER) {
        // Attain min speed
        acc = max(state.get("f_vx") - state.get("vx"), state.get("acc") - 0.5);

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

    double target_steer = target_heading - state.get("heading");
    if(target_steer > 0) {
        target_steer = min(target_steer, state.get("steer") + 0.02);
    } else {
        target_steer = max(target_steer, state.get("steer") - 0.02);
    }

    state.put("steer", target_steer);
    state.put("acc", acc);

    return state.obs;
}

HA ASP_model(State state){
    return 0;
}

Obs physicsModel(State state, double t_step){}
