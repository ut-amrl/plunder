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
double la_error_scaler = 3.0 / la_error["acc"].stddev();

double TURN_HEADING = 0.15;
double TURN_TARGET = 30;
double max_velocity = 45;

double lane_diff = 4;

double laneFinder(double y) {
    return round(y / lane_diff);
}

// Motor model matches merge.py
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
        target_acc = -0.5;

        // Attain leftmost heading
        target_heading = -TURN_HEADING;
    } else if (ha == LANE_RIGHT) {
        target_acc = -0.5;

        // Attain rightmost heading
        target_heading = TURN_HEADING;
    }

    double target_steer = target_heading - state.get("heading");
    if(target_steer > state.get("steer")) {
        target_steer = min(target_steer, state.get("steer") + 0.08);
    } else {
        target_steer = max(target_steer, state.get("steer") - 0.08);
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

    // HA ha = state.ha;
    // double x = state.get("x");
    // double l_x = state.get("l_x");
    // double f_x = state.get("f_x");
    // double r_x = state.get("r_x");
    // double vx = state.get("vx");
    // double l_vx = state.get("l_vx");
    // double f_vx = state.get("f_vx");
    // double r_vx = state.get("r_vx");


    // if (ha == FASTER && sample(logistic2(f_vx, 1000000000.000000, 1.000000)))
    //     return LANE_LEFT;
    // if (ha == FASTER && sample(logistic2(DividedBy(Minus(x, r_x), vx), -1.028642, -35.895672)))
    //     return LANE_RIGHT;
    // if (ha == FASTER && sample(logistic2(Minus(r_x, f_x), -38.665745, 0.395329)))
    //     return SLOWER;
    // if (ha == LANE_LEFT && sample(logistic2(f_x, 1000000000.000000, 1.000000)))
    //     return FASTER;
    // if (ha == LANE_LEFT && sample(logistic2(f_x, 1000000000.000000, 1.000000)))
    //     return LANE_RIGHT;
    // if (ha == LANE_LEFT && sample(logistic2(vx, 1000000000.000000, 1.000000)))
    //     return SLOWER;
    // if (ha == LANE_RIGHT && sample(logistic2(Minus(r_x, x), 8.462489, -0.367737)))
    //     return FASTER;
    // if (ha == LANE_RIGHT && sample(logistic2(r_vx, 1000000000.000000, 1.000000)))
    //     return LANE_LEFT;
    // if (ha == LANE_RIGHT && sample(logistic2(DividedBy(Minus(x, r_x), l_vx), -0.377488, 10.546384)))
    //     return SLOWER;
    // if (ha == SLOWER && sample(logistic2(Minus(x, f_x), -39.469845, -0.228033)))
    //     return FASTER;
    // if (ha == SLOWER && sample(logistic2(vx, 1000000000.000000, 1.000000)))
    //     return LANE_LEFT;
    // if (ha == SLOWER && sample(logistic2(Minus(x, r_x), -43.102879, -0.396746)))
    //     return LANE_RIGHT;

    // return ha;
}

Obs physicsModel(State state, double t_step){}
