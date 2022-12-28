#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;


// ACTION-SELECTION POLICY: Transition to new high-level action based on current action and state
// ----- Curated Selection of ASPs ---------------------------------------------

/*
* This is a probabilistic hand-crafted action-selection policy.
*/
HA ASP_model(State state){
    HA ha = state.ha;

    double xToTarget = state.get("target") - state.get("pos");                            // distance to the target

    // Probabilistic
    bool cond1 = flip(logistic(0, 2.5, state.get("vel") - state.get("vMax")));
    bool cond2 = flip(logistic(0, -1, xToTarget - DistTraveled(state.get("vel"), state.get("decMax"))));

    // Deterministic
    // bool cond1 = state.get("target") - state.get("vMax") >= 0;                                     // is at max velocity (can no longer accelerate)
    // bool cond2 = xToTarget - DistTraveled(state.get("vel"), state.get("decMax")) < EPSILON;  // needs to decelerate or else it will pass target


    if(ha == ACC){                          // transitions starting with ACC
        if(cond1 && !cond2) ha=CON;         // ACC -> CON (expected)
        else if(cond2) ha=DEC;              // ACC -> DEC (expected)
        else ha=ACC;                           // "no transition" is the default
    }
    else if(ha == CON){                          // transitions starting with CON
        // if(!cond1 && !cond2) ha=ACC;        // CON -> ACC (rare)
        if(false) ha=ACC;        // CON -> ACC (rare)
        else if(cond2) ha=DEC;              // CON -> DEC (expected)
        else ha=CON;                           // "no transition" is the default
    }
    else if(ha == DEC){
        // if(!cond1 && !cond2) ha=ACC;        // DEC -> ACC (0)
        // else if(cond1 && !cond2) ha=CON;    // DEC -> CON (0)
        if(false) ha=ACC;        // DEC -> ACC (0)
        else if(false) ha=CON;    // DEC -> CON (0)
        else ha=DEC;                           // "no transition" is the default
    }

    return ha;
}

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
// Motor model parameters:
const double JERK = 2;                  // rate of change of acceleration (jerk)
const double JERK_ERROR = 0.3;          // additional low-level action error standard deviation while transitioning

map<string, normal_distribution<double>> la_error = {
    { "acc", normal_distribution<double>(0.0, 0.1) }
};

Obs motorModel(State state, bool error){
    HA ha = state.ha;

    double change = JERK;
    if(error){
        change += (la_error["acc"])(gen) * (JERK_ERROR / la_error["acc"].stddev());
    }
    
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



// PHYSICS SIM: Given a current high-level action, apply a motor controller and update observed state. Runs once per time step
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

    return state.obs;
}