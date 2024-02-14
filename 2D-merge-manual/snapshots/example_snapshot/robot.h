#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "steer", normal_distribution<double>(0.0, 0.02) },
    { "acc", normal_distribution<double>(0.0, 2) }
};
double la_error_scaler = 1.0;

double TURN_HEADING = 0.1;
double TURN_TARGET = 30;
double max_velocity = 25;
double min_velocity = 15;

double lane_diff = 4;

double laneFinder(double y) {
    return round(y / lane_diff);
}

// Motor model matches highway2d.py
Obs motorModel(State state, bool error){
    HA ha = state.ha;

    double target_acc = 0.0;
    double target_steer = 0.0;
    
    if(ha == FASTER) {
        // Attain max speed
        target_acc = 4.0;

        // Follow current lane
        double target_y = laneFinder(state.get("y")) * lane_diff;
        double target_heading = atan((target_y - state.get("y")) / TURN_TARGET);
        target_steer = max(min(target_heading - state.get("heading"), 0.02), -0.02);
    } else if (ha == SLOWER) {
        // Attain min speed
        target_acc = -4.0;

        // Follow current lane
        double target_y = laneFinder(state.get("y")) * lane_diff;
        double target_heading = atan((target_y - state.get("y")) / TURN_TARGET);
        target_steer = max(min(target_heading - state.get("heading"), 0.02), -0.02);
    } else if (ha == LANE_LEFT) {
        target_acc = 0;
        target_steer = max(min(-TURN_HEADING - state.get("heading"), 0.0), -0.03);
    } else if (ha == LANE_RIGHT) {
        target_acc = 0;
        target_steer = max(min(TURN_HEADING - state.get("heading"), 0.03), 0.0);
    }

    state.put("steer", target_steer);
    state.put("acc", target_acc);

    return state.obs;
}

HA ASP_model(State state){
    bool front_clear = flip(logistic(1, 40, (state.get("f_x") - state.get("x")) / state.get("vx")));

    if(state.ha == LANE_RIGHT) {
        bool right_clear = flip(logistic(0.5, 40, (state.get("r_x") - state.get("x")) / state.get("vx")));
        if(right_clear)
            return LANE_RIGHT;
        if(front_clear) 
            return FASTER;

        return SLOWER;
    }

    bool right_clear = flip(logistic(1, 40, (state.get("r_x") - state.get("x")) / state.get("vx")));
    if(right_clear)
        return LANE_RIGHT;
    if(front_clear) 
        return FASTER;

    return SLOWER;
}

Obs physicsModel(State state, double t_step){}
