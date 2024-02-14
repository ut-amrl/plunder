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

double TURN_HEADING = 0.15;
double TURN_TARGET = 30;
double max_velocity = 40;
double turn_velocity = 30;

double lane_diff = 4;

double laneFinder(double y) {
    return round(y / lane_diff);
}

// Motor model matches highway2d.py
Obs motorModel(State state, bool error){
    HA ha = state.ha;

    double target_acc = 0.0;
    double target_heading = 0.0;
    
    if(ha == FASTER) {
        // Attain max speed
        target_acc = max_velocity - state.get("vx");

        // Follow current lane
        double target_y = laneFinder(state.get("y")) * lane_diff;
        target_heading = atan((target_y - state.get("y")) / TURN_TARGET);
    } else if (ha == SLOWER) {
        // Attain min speed
        target_acc = state.get("f_vx") - state.get("vx");

        // Follow current lane
        double target_y = laneFinder(state.get("y")) * lane_diff;
        target_heading = atan((target_y - state.get("y")) / TURN_TARGET);
    } else if (ha == LANE_LEFT) {
        target_acc = turn_velocity - state.get("vx");

        // Attain leftmost heading
        target_heading = -TURN_HEADING;
    } else if (ha == LANE_RIGHT) {
        target_acc = turn_velocity - state.get("vx");

        // Attain rightmost heading
        target_heading = TURN_HEADING;
    }

    double target_steer = target_heading - state.get("heading");
    if(target_steer > state.get("steer")) {
        target_steer = min(target_steer, state.get("steer") + 0.04);
    } else {
        target_steer = max(target_steer, state.get("steer") - 0.04);
    }

    if(target_acc > state.get("acc")) {
        target_acc = min(target_acc, state.get("acc") + 4);
    } else {
        target_acc = max(target_acc, state.get("acc") - 6);
    }

    state.put("steer", target_steer);
    state.put("acc", target_acc);

    // state.put("acc", state.get("acc") + (la_error["acc"])(gen));
    // state.put("steer", state.get("steer") + (la_error["steer"])(gen));

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

    // HA ha = state.ha;
    // double x = state.get("x");
    // double l_x = state.get("l_x");
    // double f_x = state.get("f_x");
    // double r_x = state.get("r_x");
    // double vx = state.get("vx");
    // double l_vx = state.get("l_vx");
    // double f_vx = state.get("f_vx");
    // double r_vx = state.get("r_vx");

    // if (ha == FASTER && And(sample(logistic2(Minus(r_x, l_x), -42.325607, -11.322358)), sample(logistic2(Minus(x, f_x), -39.853004, 0.471738))))
    //     return LANE_LEFT;
    // if (ha == FASTER && And(sample(logistic2(Minus(r_x, f_x), 2.268923, 1.691306)), sample(logistic2(Minus(x, f_x), -37.500061, 0.498594))))
    //     return LANE_RIGHT;
    // if (ha == FASTER && sample(logistic2(DividedBy(Minus(f_x, x), vx), 0.977072, -31.665253)))
    //     return SLOWER;
    // if (ha == LANE_LEFT && sample(logistic2(DividedBy(Minus(f_x, x), r_vx), 2.090207, 93.243362)))
    //     return FASTER;
    // if (ha == LANE_LEFT && sample(logistic2(f_vx, 1000000000.000000, 1.000000)))
    //     return LANE_RIGHT;
    // if (ha == LANE_LEFT && sample(logistic2(Minus(l_x, x), 30.264160, -3.164596)))
    //     return SLOWER;
    // if (ha == LANE_RIGHT && sample(logistic2(DividedBy(Minus(f_x, x), vx), 1.538538, 14.793274)))
    //     return FASTER;
    // if (ha == LANE_RIGHT && sample(logistic2(r_vx, 1000000000.000000, 1.000000)))
    //     return LANE_LEFT;
    // if (ha == LANE_RIGHT && sample(logistic2(DividedBy(Minus(x, r_x), vx), -0.984751, 182.378754)))
    //     return SLOWER;
    // if (ha == SLOWER && sample(logistic2(DividedBy(Minus(f_x, x), vx), 1.495547, 73.774384)))
    //     return FASTER;
    // if (ha == SLOWER && sample(logistic2(Minus(l_x, f_x), 5.026824, 4.910437)))
    //     return LANE_LEFT;
    // if (ha == SLOWER && sample(logistic2(Minus(r_x, f_x), 1.479489, 1.003578)))
    //     return LANE_RIGHT;
    // return ha;
}

Obs physicsModel(State state, double t_step){}
