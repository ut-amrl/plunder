#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "steer", normal_distribution<double>(0.0, 0.01) },
    { "acc", normal_distribution<double>(0.0, 2) }
};
double la_error_scaler = 1.0;

double TURN_HEADING = 0.1;
double TURN_TARGET = 30;
double max_velocity = 25;
double min_velocity = 20;

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
        target_acc = 4;

        // Follow current lane
        double target_y = laneFinder(state.get("y")) * lane_diff;
        double target_heading = atan((target_y - state.get("y")) / TURN_TARGET);
        target_steer = max(min(target_heading - state.get("heading"), 0.015), -0.015);
    } else if (ha == SLOWER) {
        // Attain min speed
        target_acc = -4;

        // Follow current lane
        double target_y = laneFinder(state.get("y")) * lane_diff;
        double target_heading = atan((target_y - state.get("y")) / TURN_TARGET);
        target_steer = max(min(target_heading - state.get("heading"), 0.015), -0.015);
    } else if (ha == LANE_LEFT) {
        target_acc = 4;
        target_steer = -0.02;
    } else if (ha == LANE_RIGHT) {
        target_acc = 4;
        target_steer = 0.02;
    }

    if(state.get("vx") >= max_velocity - 0.01) {
        target_acc = min(target_acc, 0.0);
    }
    if(state.get("vx") <= min_velocity + 0.01) {
        target_acc = max(target_acc, 0.0);
    }

    if(target_steer > 0) {
        target_steer = min(target_steer, TURN_HEADING - state.get("heading"));
    }
    if(target_steer < 0) {
        target_steer = max(target_steer, -TURN_HEADING - state.get("heading"));
    }

    state.put("steer", target_steer);
    state.put("acc", target_acc);

    return state.obs;
}

HA ASP_model(State state){
    double l_x = state.get("l_x") - state.get("x");
    double f_x = state.get("f_x") - state.get("x");
    double r_x = state.get("r_x") - state.get("x");

    bool front_clear;
    if(state.ha == FASTER) {
        front_clear = flip(logistic(1, 30, f_x / state.get("vx")));
    } else {
        front_clear = flip(logistic(1.5, 30, f_x / state.get("vx")));
    }
    bool left_clear = flip(logistic(1, 30, l_x / state.get("vx")));
    bool right_clear = flip(logistic(1, 30, r_x / state.get("vx")));
    bool left_better = flip(logistic(0, 1, l_x - r_x));

    if(front_clear)
        return FASTER;
    else if (state.ha != LANE_RIGHT && left_clear && left_better)
        return LANE_LEFT;
    else if (state.ha != LANE_LEFT && right_clear)
        return LANE_RIGHT;
    return SLOWER;
}

Obs physicsModel(State state, double t_step){}
