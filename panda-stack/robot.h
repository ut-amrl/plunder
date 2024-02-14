#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "vx", normal_distribution<double>(0.0, 0.3) },
    { "vy", normal_distribution<double>(0.0, 0.3) },
    { "vz", normal_distribution<double>(0.0, 0.3) },
    { "end", normal_distribution<double>(0.0, 0.3) }
};
double la_error_scaler = 1.0;

double bound(double input) {
    return min(max(input, -1.0), 1.0);
}

// Custom-defined motor models
Obs motorModel(State state, bool error){
    HA ha = state.ha;
    double bx1 = state.get("bx1"),
        by1 = state.get("by1"),
        bz1 = state.get("bz1"),
        bx2 = state.get("bx2"),
        by2 = state.get("by2"),
        bz2 = state.get("bz2"),
        tx2 = state.get("tx2"),
        ty2 = state.get("ty2"),
        tz2 = state.get("tz2") + 0.01;

    double vx, vy, vz, end;
    if(ha == MOVE_TO_CUBE_BOTTOM) {
        vx = 4 * bx1, vy = 4 * by1, vz = 4 * bz1, end = 1;
    } else if (ha == MOVE_TO_TARGET) {
        vx = 4 * tx2, vy = 4 * ty2, vz = 4 * tz2, end = -1;
        if (vz < 0) {
            vz = max(state.get("vz") - 0.15, vz);
        }
    } else if (ha == LIFT) {
        vx = 0, vy = 0, vz = 0.5, end = 1;
    } else if (ha == MOVE_TO_CUBE_TOP) {
        vx = 4 * bx2, vy = 4 * by2, vz = 4 * bz2, end = 1;
        if (vz < 0) {
            vz = max(state.get("vz") - 0.15, vz);
        }
    } else if (ha == GRASP) {
        vx = 0, vy = 0, vz = 0.5, end = -1;
    } else {
        throw invalid_argument("Invalid high-level action label");
    }
    
    state.put("vx", vx);
    state.put("vy", vy);
    state.put("vz", vz);
    state.put("end", end);

    // state.put("vx", state.get("vx") + (la_error["vx"])(gen));
    // state.put("vy", state.get("vy") + (la_error["vy"])(gen));
    // state.put("vz", state.get("vz") + (la_error["vz"])(gen));
    // state.put("end", state.get("end") + (la_error["end"])(gen));

    return state.obs;
}

HA ASP_model(State state){
    HA ha = state.ha;
    double bx1 = state.get("bx1"),
        by1 = state.get("by1"),
        bz1 = state.get("bz1"),
        bx2 = state.get("bx2"),
        by2 = state.get("by2"),
        bz2 = state.get("bz2"),
        tx2 = state.get("tx2"),
        ty2 = state.get("ty2"),
        tz2 = state.get("tz2");
    double x = state.get("x"), y = state.get("y"), z = state.get("z");

    if(ha == MOVE_TO_CUBE_BOTTOM && bz1 > -0.005) {
        return MOVE_TO_TARGET;
    }
    if(ha == MOVE_TO_TARGET && abs(tx2) + abs(ty2) < 0.003) {
        return LIFT;
    }
    if(ha == LIFT && flip(logistic(0.15, 200, state.get("z")))) {
        return MOVE_TO_CUBE_TOP;
    }
    if(ha == MOVE_TO_CUBE_TOP && bz2 > -0.005) {
        return GRASP;
    }
    if(ha == GRASP && flip(logistic(0.15, 200, state.get("z")))) {
        return MOVE_TO_TARGET;
    }
    return ha;

    
    // if (ha == GRASP && sample(logistic(-0.100266, -80.894936, tz2)))
    //     return MOVE_TO_TARGET;
    // else if (ha == LIFT && sample(logistic(0.6139, 41.856, bz1 + ty2)))
    //     return MOVE_TO_CUBE_BOTTOM;
    // else if (ha == LIFT && sample(logistic(-0.142514, -103.338959, bz1)))
    //     return MOVE_TO_CUBE_TOP;
    // else if (ha == MOVE_TO_CUBE_BOTTOM && sample(logistic(0.4281, 116.65596, ty2)))
    //     return LIFT;
    // else if (ha == MOVE_TO_CUBE_BOTTOM && sample(logistic(-0.332, -20.043522, z)))
    //     return MOVE_TO_CUBE_TOP;
    // else if (ha == MOVE_TO_CUBE_BOTTOM && sample(logistic(0.02998, 29400.211, bz1 + tz2)))
    //     return MOVE_TO_TARGET;
    // else if (ha == MOVE_TO_CUBE_TOP && sample(logistic(-0.009927, 83562.8, bz2 + bz2)))
    //     return GRASP;
    // else if (ha == MOVE_TO_CUBE_TOP && sample(logistic(0.176457, 44.735584, tz2)))
    //     return MOVE_TO_CUBE_BOTTOM;
    // else if (ha == MOVE_TO_TARGET && sample(logistic(0.002998, -41319.92, abs(tx2) + abs(ty2))))
    //     return LIFT;
    // return ha;
}

Obs physicsModel(State state, double t_step){}
