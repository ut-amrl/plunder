#pragma once

#include "utils.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;


// ACTION-SELECTION POLICY: Transition to new high-level action based on current action and state
// ----- Curated Selection of ASPs ---------------------------------------------

/*
* This is a hand-crafted, non-probabilistic action-selection policy.
*/
HA ASP_Hand(State state){
    HA ha = state.ha;

    double xToTarget = state.get("target") - state.get("pos");                                  // distance to the target

    bool cond1 = state.get("target") - state.get("vMax") >= 0;                                     // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget - DistTraveled(state.get("vel"), state.get("decMax")) < EPSILON;  // needs to decelerate or else it will pass target

    if(ha == ACC){                          // transitions starting with ACC
        if(cond1 && !cond2) ha=CON;         // ACC -> CON (expected)
        else if(cond2) ha=DEC;              // ACC -> DEC (expected)
        else ha=ACC;                           // "no transition" is the default
    }
    else if(ha == CON){                          // transitions starting with CON
        // if(!cond1 && !cond2) ha=ACC;        // CON -> ACC (rare)
        if(false) ha=ACC;
        else if(cond2) ha=DEC;              // CON -> DEC (expected)
        else ha=CON;                           // "no transition" is the default
    }
    else if(ha == DEC){
        // if(!cond1 && !cond2) ha=ACC;        // DEC -> ACC (0)
        // else if(cond1 && !cond2) ha=CON;    // DEC -> CON (0)
        if(false) ha=ACC;
        else if(false) ha=CON;
        else ha=DEC;                           // "no transition" is the default
    }

    return ha;
}

/*
* This is a probabilistic hand-crafted action-selection policy.
*/
HA ASP_Hand_prob(State state){
    HA ha = state.ha;

    double xToTarget = state.get("target") - state.get("pos");                            // distance to the target

    bool cond1smooth = flip(logistic(0, 2.5, state.get("vel") - state.get("vMax")));
    bool cond2smooth = flip(logistic(0, -1, xToTarget - DistTraveled(state.get("vel"), state.get("decMax"))));

    if(ha == ACC){                          // transitions starting with ACC
        if(cond1smooth && !cond2smooth) ha=CON;         // ACC -> CON (expected)
        else if(cond2smooth) ha=DEC;              // ACC -> DEC (expected)
        else ha=ACC;                           // "no transition" is the default
    }
    else if(ha == CON){                          // transitions starting with CON
        // if(!cond1smooth && !cond2smooth) ha=ACC;        // CON -> ACC (rare)
        if(false) ha=ACC;        // CON -> ACC (rare)
        else if(cond2smooth) ha=DEC;              // CON -> DEC (expected)
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

/*
* This is a uniformly random ASP.
*/
HA ASP_random(State state){
    return rand() % numHA;
}

// Select an ASP to use
vector<asp*> ASPs = { &ASP_Hand,              // 0
                      &ASP_Hand_prob,         // 1
                      &ASP_random,            // 2
                    };

asp* ASP_model(int model){
    return ASPs[model];
}

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
// TODO: deal with error (abstract away std deviation) and deal with different stddev for different variables
normal_distribution<double> la_error = normal_distribution<double>(MEAN_ERROR, STDDEV_ERROR);
Obs motorModel(State state, bool error){
    HA ha = state.ha;

    double change = JERK;
    if(error){
        change += la_error(gen) * (JERK_ERROR / STDDEV_ERROR);
    }
    
    if(ha == ACC){
        state.put("acc", min(state.get("acc") + change, state.get("accMax")));
    } else if (ha == DEC) {
        state.put("acc", max(state.get("acc") - change, state.get("decMax")));
    } else {
        if(state.get("acc") < 0)
            state.put("acc", min(state.get("acc") + change, state.get("accMax")));
        if(state.get("acc") > 0)
            state.put("acc", max(state.get("acc") - change, state.get("decMax")));
    }

    // Induce some additional lesser error
    if(error){
        state.put("acc", state.get("acc") + la_error(gen));
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