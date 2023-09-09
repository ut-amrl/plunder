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
        tz2 = state.get("tz2") - state.get("z") + 0.01;

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
    double dx = min(0.03, abs(vx / 4));
    vx = min(max(vx, state.get("vx") - dx), state.get("vx") + dx);
    double dy = min(0.03, abs(vy / 4));
    vy = min(max(vy, state.get("vy") - dy), state.get("vy") + dy);
    double dz = min(0.03, abs(vz / 4));
    vz = min(max(vz, state.get("vz") - dz), state.get("vz") + dz);
    double dend = min(0.03, abs(end / 4));
    end = min(max(end, state.get("end") - dend), state.get("end") + dend);
    
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
        tz2 = state.get("tz2") - state.get("z") + 0.01;
    
    if(ha == MOVE_TO_CUBE_BOTTOM && abs(bx1) < 0.002 && abs(by1) < 0.002 && abs(bz1) < 0.006) {
        return MOVE_TO_TARGET;
    }
    if(ha == MOVE_TO_TARGET && abs(tx2) < 0.002 && abs(ty2) < 0.002 && abs(tz2) < 0.002) {
        return LIFT;
    }
    if(ha == LIFT && state.get("z") > 0.1 && state.get("bz2") < 0.03) {
        return MOVE_TO_CUBE_TOP;
    }
    if(ha == MOVE_TO_CUBE_TOP && abs(bx2) < 0.002 && abs(by2) < 0.002 && abs(bz2) < 0.006) {
        return GRASP;
    }
    if(ha == GRASP && state.get("z") > 0.1) {
        return MOVE_TO_TARGET;
    }
    return ha;
}

Obs physicsModel(State state, double t_step){}
