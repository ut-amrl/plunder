#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "vx", normal_distribution<double>(0.0, 0.01) },
    { "vy", normal_distribution<double>(0.0, 0.01) },
    { "vz", normal_distribution<double>(0.0, 0.01) },
    { "end", normal_distribution<double>(0.0, 0.01) }
};
double la_error_scaler = 1.0;

double bound(double input) {
    return min(max(input, -1.0), 1.0);
}

// Custom-defined motor models
Obs motorModel(State state, bool error){
    HA ha = state.ha;
    double bx1 = state.get("bx1") - state.get("x"),
        by1 = state.get("by1") - state.get("y"),
        bz1 = state.get("bz1") - state.get("z"), 
        bx2 = state.get("bx2") - state.get("x"), 
        by2 = state.get("by2") - state.get("y"), 
        bz2 = state.get("bz2") - state.get("z"),
        tx2 = state.get("tx2") - state.get("x"),
        ty2 = state.get("ty2") - state.get("y"), 
        tz2 = state.get("tz2") - state.get("z") + 0.02;

    double vx, vy, vz, end;
    if(ha == MOVE_TO_CUBE_BOTTOM) {
        vx = 4 * bx1, vy = 4 * by1, vz = 4 * bz1, end = 1;
    } else if (ha == MOVE_TO_TARGET) {
        vx = 4 * tx2, vy = 4 * ty2, vz = 4 * tz2, end = -1;
    } else if (ha == LIFT) {
        vx = 0, vy = 0, vz = 0.5;, end = 1;
    } else if (ha == MOVE_TO_CUBE_TOP) {
        vx = 4 * bx2, vy = 4 * by2, vz = 4 * bz2, end = 1;
    } else if (ha == GRASP) {
        vx = 0, vy = 0, vz = 0.5, end = -1;
    } else {
        throw invalid_argument("Invalid high-level action label");
    }

    // Make actions continuous
    vx = min(max(vx, state.get("vx") - 0.02), state.get("vx") + 0.02);
    vy = min(max(vy, state.get("vy") - 0.02), state.get("vy") + 0.02);
    vz = min(max(vz, state.get("vz") - 0.02), state.get("vz") + 0.02);
    end = min(max(end, state.get("end") - 0.02), state.get("end") + 0.02);
    
    return state.obs;
}

HA ASP_model(State state){
    HA ha = state.ha;
    double bx1 = state.get("bx1") - state.get("x"),
        by1 = state.get("by1") - state.get("y"),
        bz1 = state.get("bz1") - state.get("z"), 
        bx2 = state.get("bx2") - state.get("x"), 
        by2 = state.get("by2") - state.get("y"), 
        bz2 = state.get("bz2") - state.get("z"),
        tx2 = state.get("tx2") - state.get("x"),
        ty2 = state.get("ty2") - state.get("y"), 
        tz2 = state.get("tz2") - state.get("z") + 0.02;
    
    if(ha == MOVE_TO_CUBE_BOTTOM && abs(bx1) < 0.003 && abs(by1) < 0.003 && abs(bz1) < 0.005) {
        return MOVE_TO_TARGET;
    }
    if(ha == MOVE_TO_TARGET && abs(tx2) < 0.002 && abs(ty2) < 0.002) {
        return LIFT;
    }
    if(ha == LIFT && state.get("z") > 0.1 && state.get("bz2") < 0.03) {
        return MOVE_TO_CUBE_TOP;
    }
    if(ha == MOVE_TO_CUBE_TOP && abs(bx2) < 0.003 && abs(by2) < 0.003 && abs(bz2) < 0.005) {
        return GRASP;
    }
    if(ha == GRASP && state.get("z") > 0.1) {
        return MOVE_TO_TARGET;
    }
    return ha;
}

Obs physicsModel(State state, double t_step){}
