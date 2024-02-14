#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;


// ACTION-SELECTION POLICY: Transition to new high-level action based on current action and state
// ----- Curated Selection of ASPs ---------------------------------------------

// Return the distance this robot would travel before stopping if it began decelerating immediately
double DistTraveled(double v, double dec){
    return - v * v / (2 * dec);
}

/*
* This is a probabilistic hand-crafted action-selection policy.
*/
HA ASP_model(State state){
    HA ha = state.ha;

    // Probabilistic
    bool cond1 = flip(logistic(0, 1, state.get("vel") - state.get("vMax"))); // Velocity exceeds maximum velocity
    bool cond2 = flip(logistic(state.get("target"), 0.5, state.get("pos") + DistTraveled(state.get("vel"), state.get("decMax")))); // Distance traveled exceeds target

    if(ha == ACC){                // transitions starting with ACC
        if(cond1 && !cond2) ha=CON;         // ACC -> CON
        else if(cond2) ha=DEC;              // ACC -> DEC
    } else if(ha == CON){         // transitions starting with CON
        if(cond2) ha=DEC;                   // CON -> DEC
    }

    // if (ha == ACC && sample(logistic2(Minus(state.get("vel"), state.get("vMax")), -0.456573, 1.388253)))
    //     return CON;
    // if (ha == ACC && sample(logistic2(Minus(state.get("dns"), DistTraveled(state.get("vel"), state.get("decMax"))), -6.926392, -2.609171)))
    //     return DEC;
    // if (ha == CON && sample(logistic2(Minus(DistTraveled(state.get("vel"), state.get("decMax")), state.get("dns")), -2.829544, 0.814321)))
    //     return DEC;

    return ha;
}

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
// Motor model parameters:
map<string, normal_distribution<double>> la_error = {
    { "acc", normal_distribution<double>(0.0, 0.5) } // acceleration
};
double la_error_scaler = 1.0;

Obs motorModel(State state, bool error){
    HA ha = state.ha;

    double change = 1;

    // Linearly increment / decrement acceleration to match high-level action
    if(ha == ACC){
        state.put("acc", min(state.get("acc") + change, state.get("accMax")));
    } else if (ha == DEC) {
        state.put("acc", max(state.get("acc") - change, state.get("decMax")));
    } else {
        if(state.get("acc") < 0)
            state.put("acc", min(0.0, state.get("acc") + change));
        if(state.get("acc") > 0)
            state.put("acc", max(0.0, state.get("acc") - change));
    }

    // Induce some additional lesser error
    if(error){
        state.put("acc", state.get("acc") + (la_error["acc"])(gen));
    }

    return state.obs;
}


// PHYSICS SIM: Given a current low-level action, update observed state. Runs once per time step
Obs physicsModel(State state, double t_step){
    double vPrev = state.get("vel");
    double xPrev = state.get("pos");
    
    // Update velocity and displacement accordingly
    state.put("vel", vPrev + state.get("acc") * t_step);

    if(state.get("vel") < EPSILON){ // Round to 0
        state.put("vel", 0);
    }

    if(abs(state.get("vel") - state.get("vMax")) < EPSILON){ // Round to vMax
        state.put("vel", state.get("vMax"));
    }

    if(abs(state.get("pos") - state.get("target")) < EPSILON){ // Round to target
        state.put("pos", state.get("target"));
    }

    state.put("pos", xPrev + (state.get("vel") + vPrev) / 2 * t_step);
    state.put("dns", state.get("target") - state.get("pos"));

    return state.obs;
}