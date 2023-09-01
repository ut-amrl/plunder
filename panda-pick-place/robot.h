#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "vx", normal_distribution<double>(0.0, 0.001) },
    { "vy", normal_distribution<double>(0.0, 0.001) },
    { "vz", normal_distribution<double>(0.0, 0.001) },
    { "end", normal_distribution<double>(0.0, 0.001) }
};
double la_error_scaler = 1.0;

// Motor model matches highway2d.py
Obs motorModel(State state, bool error){
    HA ha = state.ha;

    if(ha == MOVE_TO_CUBE) {
        state.put("vx", 5 * state.get("bx"));
        state.put("vy", 5 * state.get("by"));
        state.put("vz", 5 * state.get("bz"));
        state.put("end", 1);
    } else if (ha == GRASP) {
        state.put("vx", 0);
        state.put("vy", 0);
        state.put("vz", 0);
        state.put("end", -1);
    } else if (ha == MOVE_TO_TARGET) {
        state.put("vx", 5 * state.get("tx"));
        state.put("vy", 5 * state.get("ty"));
        state.put("vz", 5 * state.get("tz"));
        state.put("end", -1);
    } else {
        throw new IllegalArgumentException();
    }

    return state.obs;
}

HA ASP_model(State state){
    HA ha = state.ha;
    if(ha == MOVE_TO_CUBE && abs(state.get("bx")) < 0.01 && abs(state.get("by")) < 0.015 && abs(state.get("bz")) < 0.01) {
        return GRASP;
    }
    if(ha == GRASP && state.get("end_width") < 0.04) {
        return MOVE_TO_TARGET;
    }
    return ha;
}

Obs physicsModel(State state, double t_step){}
