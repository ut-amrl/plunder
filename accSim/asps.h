#pragma once

#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>

#include "robot.h"
#include "settings.h"

using namespace std;

typedef HA asp_t(HA, Obs, Robot&);


// ----- Helper Methods ---------------------------------------------

/*
 * Randomly transitions to an incorrect high-level action with specified probability
 */
HA pointError(HA prevHA, HA ha, Robot& r, bool useSafePointError){
    if(usePointError){
        int haDif = 0;
        if(!r.sampleDiscrete(r.pointAccuracy)){
            if(r.sampleDiscrete(0.5)) haDif = 1;
            else haDif = 2;
        }
        ha = static_cast<HA>((ha + haDif)%3);
        if(useSafePointError){
            if(prevHA == CON && ha == ACC) ha = DEC;
            if(prevHA == DEC) ha = DEC;
        }
    }
    return ha;
}


double logistic(double x_0, double a, double x){
    return 1.0 / (1.0 + exp(-a * (x - x_0)));
}

// ----- Curated Selection of ASPs ---------------------------------------------

/*
 * This is a hand-crafted, non-probabilistic action-selection policy.
 */
HA ASP_Hand(HA ha, Obs state, Robot& r){
    double xToTarget = r.target - state.pos;                                  // distance to the target

    bool cond1 = state.vel - r.vMax >= 0;                                     // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget - r.DistTraveled(state.vel, r.decMax) < robotEpsilon;  // needs to decelerate or else it will pass target

    if(ha == ACC){                          // transitions starting with ACC
        if(cond1 && !cond2) ha=CON;         // ACC -> CON (expected)
        else if(cond2) ha=DEC;              // ACC -> DEC (expected)
        else ACC;                           // "no transition" is the default
    }
    if(ha == CON){                          // transitions starting with CON
        // if(!cond1 && !cond2) ha=ACC;        // CON -> ACC (rare)
        if(false) ha=ACC;
        else if(cond2) ha=DEC;              // CON -> DEC (expected)
        else CON;                           // "no transition" is the default
    }
    if(ha == DEC){
        // if(!cond1 && !cond2) ha=ACC;        // DEC -> ACC (0)
        // else if(cond1 && !cond2) ha=CON;    // DEC -> CON (0)
        if(false) ha=ACC;
        else if(false) ha=CON;
        else DEC;                           // "no transition" is the default
    }

    return ha;
}

/*
 * This is a probabilistic hand-crafted action-selection policy.
 */
HA ASP_Hand_prob(HA ha, Obs state, Robot& r){
    double xToTarget = r.target - state.pos;                            // distance to the target

    bool cond1 = state.vel - r.vMax >= 0;                                     // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget - r.DistTraveled(state.vel, r.decMax) < robotEpsilon;  // needs to decelerate or else it will pass target

    bool cond1smooth = r.sampleDiscrete(logistic(0, 2.5, state.vel - r.vMax));
    bool cond2smooth = r.sampleDiscrete(logistic(0, -1, xToTarget - r.DistTraveled(state.vel, r.decMax)));

    if(ha == ACC){                          // transitions starting with ACC
        if(cond1smooth && !cond2smooth) ha=CON;         // ACC -> CON (expected)
        else if(cond2smooth) ha=DEC;              // ACC -> DEC (expected)
        else ACC;                           // "no transition" is the default
    }
    if(ha == CON){                          // transitions starting with CON
        // if(!cond1smooth && !cond2smooth) ha=ACC;        // CON -> ACC (rare)
        if(false) ha=ACC;        // CON -> ACC (rare)
        else if(cond2smooth) ha=DEC;              // CON -> DEC (expected)
        else CON;                           // "no transition" is the default
    }
    if(ha == DEC){
        // if(!cond1 && !cond2) ha=ACC;        // DEC -> ACC (0)
        // else if(cond1 && !cond2) ha=CON;    // DEC -> CON (0)
        if(false) ha=ACC;        // DEC -> ACC (0)
        else if(false) ha=CON;    // DEC -> CON (0)
        else DEC;                           // "no transition" is the default
    }
    return ha;
}

/*
 * This is a uniformly random ASP.
 */
HA ASP_random(HA ha, Obs state, Robot& r){
    if(r.sampleDiscrete(0.33)){
        ha = ACC;
    } else if (r.sampleDiscrete(0.5)){
        ha = DEC;
    } else {
        ha = CON;
    }
    return ha;
}

// Select an ASP to use
vector<asp_t*> ASPs = { &ASP_Hand,              // 0
                        &ASP_Hand_prob,         // 1
                        &ASP_random,            // 2
                      };

asp_t* ASP_model(int model){
    return ASPs[model];
}