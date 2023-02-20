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

double KP_H = 0.5;
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
        acc = 0.4 * (max_velocity - state.get("vx"));

        // Follow current lane
        double target_y = laneFinder(state.get("y")) * lane_diff;
        target_heading = atan((target_y - state.get("y")) / TURN_TARGET);
    } else if (ha == SLOWER) {
        // Attain min speed
        acc = state.get("f_vx") - state.get("vx");

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
    state.put("acc", acc);

    return state.obs;
}

HA ASP_model(State state){
    HA ha;

    bool front_clear = flip(logistic(30, 0.5, state.get("f_x") - state.get("x")));
    bool left_clear = flip(logistic(30, 0.5, state.get("l_x") - state.get("x")));
    bool right_clear = flip(logistic(30, 0.5, state.get("r_x") - state.get("x")));
    bool left_better = flip(logistic(0, 0.5, state.get("l_x") - state.get("r_x")));

    if(front_clear) ha=FASTER; // No car in front: accelerate
    else if(left_clear && left_better) ha=LANE_LEFT; // Merge left
    else if(right_clear) ha=LANE_RIGHT; // Merge right
    else ha=SLOWER; // Nowhere to go: slow down

    return ha;
}

Obs physicsModel(State state, double t_step){}
