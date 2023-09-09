#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "vx", normal_distribution<double>(0.0, 0.4) },
    { "vy", normal_distribution<double>(0.0, 0.4) },
    { "vz", normal_distribution<double>(0.0, 0.4) },
    { "end", normal_distribution<double>(0.0, 0.4) }
};
double la_error_scaler = 1.0;

double bound(double input) {
    return min(max(x, -1), 1);
}

// Custom-defined motor models
Obs motorModel(State state, bool error){
    HA ha = state.ha;

    double vx, vy, vz, end;
    if(ha == MOVE_TO_CUBE_BOTTOM) {
        vx = 4 * (state.get("bx1") - state.get("x"));
        vy = 4 * (state.get("by1") - state.get("y"));
        vz = 4 * (state.get("bz1") - state.get("z"));
        end = 1;
    } else if (ha == MOVE_TO_TARGET) {
        vx = 4 * (state.get("tx2") - state.get("x"));
        vy = 4 * (state.get("ty2") - state.get("y"));
        vz = 4 * (state.get("tz2") - state.get("z") + 0.02);
        end = -1;
    } else if (ha == LIFT) {
        vx = 0;
        vy = 0;
        vz = 0.5;
        end = 1;
    } else if (ha == MOVE_TO_CUBE_TOP) {
        vx = 4 * (state.get("bx2") - state.get("x"));
        vy = 4 * (state.get("by2") - state.get("y"));
        vz = 4 * (state.get("bz2") - state.get("z"));
        end = 1;
    } else if (ha == GRASP) {
        vx = 0;
        vy = 0;
        vz = 0.5;
        end = -1;
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
    double bx1 = abs(state.get("bx1") - state.get("x")),
        by1 = abs(state.get("by1") - state.get("y")),
        bz1 = abs(state.get("bz1") - state.get("z")), 
        bx2 = abs(state.get("bx2") - state.get("x")), 
        by2 = abs(state.get("by2") - state.get("y")), 
        bz2 = abs(state.get("bz2") - state.get("z")),
        tx1 = abs(state.get("tx2") - state.get("x")),
        ty1 = abs(state.get("ty2") - state.get("y")), 
        tz1 = abs(state.get("tz2") - state.get("z") + 0.02);

    if(ha == MOVE_TO_CUBE_BOTTOM && bx1 < 0.003 && by1 < 0.003 && bz1 < 0.005) {
        return MOVE_TO_TARGET;
    }
    if(ha == MOVE_TO_TARGET && tx1 < 0.002 && ty1 < 0.002) {
        return LIFT;
    }
    if(ha == LIFT && state.get("z") > 0.1 && state.get("bz2") < 0.03) {
        return MOVE_TO_CUBE_TOP;
    }
    if(ha == MOVE_TO_CUBE_TOP && bx2 < 0.003 && by2 < 0.003 && bz2 < 0.005) {
        return GRASP;
    }
    if(ha == GRASP && state.get("z") > 0.1) {
        return MOVE_TO_TARGET;
    }
    return ha;
}

Obs physicsModel(State state, double t_step){}
