#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "vx", normal_distribution<double>(0.0, 0.2) },
    { "vy", normal_distribution<double>(0.0, 0.2) },
    { "vz", normal_distribution<double>(0.0, 0.2) },
    { "end", normal_distribution<double>(0.0, 0.2) }
};
double la_error_scaler = 1.0;

// Custom-defined motor models
Obs motorModel(State state, bool error){
    HA ha = state.ha;

    // RL-based
    // if(ha == MOVE_TO_CUBE) {
    //     state.put("vx", 5 * (state.get("bx") - state.get("x")));
    //     state.put("vy", 5 * (state.get("by") - state.get("y")));
    //     state.put("vz", 5 * (state.get("bz") - state.get("z")));
    //     state.put("end", 1);
    // } else if (ha == MOVE_TO_TARGET) {
    //     state.put("vx", 5 * (state.get("tx") - state.get("x")));
    //     state.put("vy", 5 * (state.get("ty") - state.get("y")));
    //     state.put("vz", 5 * (state.get("tz") - state.get("z")));
    //     state.put("end", -1);
    // } else {
    //     throw invalid_argument("Invalid high-level action label");
    // }

    // Policy-based
    if(ha == MOVE_TO_CUBE) {
        state.put("vx", 5 * (state.get("bx") - state.get("x")));
        state.put("vy", 5 * (state.get("by") - state.get("y")));
        state.put("vz", 5 * (state.get("bz") - state.get("z")));
        state.put("end", 0.6);
    } else if (ha == MOVE_TO_TARGET) {
        double t_end = -0.6;
        state.put("vx", 5 * (state.get("tx") - state.get("x")));
        state.put("vy", 5 * (state.get("ty") - state.get("y")));
        state.put("vz", 5 * (state.get("tz") - state.get("z")));

        if (state.get("end") >= -0.3) {
            t_end = -0.6;
            state.put("vx", 5 * (state.get("bx") - state.get("x")));
            state.put("vy", 5 * (state.get("by") - state.get("y")));
            state.put("vz", 5 * (state.get("bz") - state.get("z")));
        }
        if (state.get("end") >= 0) {
            t_end = -0.3;
        }
        if (state.get("end") >= 0.3) {
            t_end = 0;
        }
        if (state.get("end") >= 0.6) {
            t_end = 0.3;
        } 

        state.put("end", t_end);
    } else {
        throw invalid_argument("Invalid high-level action label");
    }

    return state.obs;
}

HA ASP_model(State state){
    HA ha = state.ha;
    
    // RL-based
    // if(ha == MOVE_TO_CUBE && abs(state.get("bx") - state.get("x")) < 0.05 && abs(state.get("by") - state.get("y")) < 0.05 && abs(state.get("bz") - state.get("z")) < 0.05) {
    //     return MOVE_TO_TARGET;
    // }

    // Policy-based
    if(ha == MOVE_TO_CUBE && 
        flip(logistic(0.0015, -5000, abs(state.get("bx") - state.get("x")))) && 
        flip(logistic(0.0015, -5000, abs(state.get("by") - state.get("y"))))) {
        return MOVE_TO_TARGET;
    }
    return ha;
}

Obs physicsModel(State state, double t_step){}
