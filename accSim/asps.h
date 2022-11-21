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
    }
    if(useSafePointError){
        if(prevHA == CON && ha == ACC) ha = CON;
        if(prevHA == DEC) ha = DEC;
    }
    return ha;
}


double logistic(double midpoint, double steepness, double input){
    return 1.0 / (1.0 + exp(-steepness * (input - midpoint)));
}

// ----- Curated Selection of ASPs ---------------------------------------------

/*
 * This is a hand-crafted action-selection policy.
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
 * Clean version of ASP_Hand, ie only containing realistic transition changes
 */
HA ASP_Hand_clean(HA ha, Obs state, Robot& r){
    double xToTarget = r.target - state.pos;                                  // distance to the target

    bool cond1 = state.vel - r.vMax >= 0;                                     // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget - r.DistTraveled(state.vel, r.decMax) < robotEpsilon;  // needs to decelerate or else it will pass target

    if(cond2){
        ha = DEC;
    }
    else if(ha == ACC && cond1){
        ha = CON;
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
 * This is a probabilistic ASP that creates a consistent error at the start (modeling the user's plan) and uses it at each timestep
 */
HA ASP_consistent_prob(HA ha, Obs state, Robot& r){
    double xToTarget = r.target - state.pos;                            // distance to the target

    bool cond1 = state.vel - r.vMax + r.cond1err >= 0;                                     // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget - r.DistTraveled(state.vel, r.decMax) + r.cond2err < robotEpsilon;  // needs to decelerate or else it will pass target

    if(ha == ACC){                          // transitions starting with ACC
        if(cond1 && !cond2) ha=CON;         // ACC -> CON (expected)
        else if(cond2) ha=DEC;              // ACC -> DEC (expected)
        else ACC;                           // "no transition" is the default
    }
    if(ha == CON){                          // transitions starting with CON
        if(!cond1 && !cond2) ha=ACC;        // CON -> ACC (rare)
        else if(cond2) ha=DEC;              // CON -> DEC (expected)
        else CON;                           // "no transition" is the default
    }
    if(ha == DEC){
        if(!cond1 && !cond2) ha=ACC;        // DEC -> ACC (0)
        else if(cond1 && !cond2) ha=CON;    // DEC -> CON (0)
        else DEC;                           // "no transition" is the default
    }
    return ha;
}

/*
 * This is a hand-crafted action-selection policy, with some perception error. Used to model perception differences in real-life demonstrations
 */ 
HA ASP_Sim(HA ha, Obs state, Robot& r){
    normal_distribution<double> distError(1, distErrorDev);
    normal_distribution<double> velError(1, velErrorDev);

    double pos = max(0.0, state.pos + distErrorMean) * distError(gen);
    double vel = (state.vel + velErrorMean) * velError(gen);

    double xToTarget = r.target - pos;                                  // distance to the target

    bool cond1 = vel - r.vMax >= 0;                                     // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget - r.DistTraveled(vel, r.decMax) < robotEpsilon;  // needs to decelerate or else it will pass target

    if(cond2){
        ha = DEC;
    }
    if(cond1 && !cond2){
        ha = CON;
    }
    if(!cond1 && !cond2){
        ha = ACC;
    }

    return ha;
}

/*
 * This is an action-selection policy generated by LDIPS, without error
 */
HA ASP_LDIPS(HA ha, Obs state, Robot& r){
    // Copy paste below
    if(ha == ACC && r.DistTraveled(state.vel, r.decMax) + state.pos - r.target >= -2.320007)
        ha = DEC;
    else if(ha == CON && r.DistTraveled(state.vel, r.decMax) - r.DistTraveled(r.vMax, r.decMax) >= -150.000000 && r.DistTraveled(r.vMax, r.decMax) + state.pos - r.target >= -0.095000)
        ha = DEC;
    else if(ha == ACC && state.vel - r.vMax >= -0.025000 && state.pos - r.target >= -499.975006)
        ha = CON;
    else if(ha == CON && r.vMax - state.vel >= 0.000000)
        ha = ACC;
    else if(ha == DEC && state.vel >= -1.000000)
        ha = DEC;
    else if(ha == ACC && state.pos >= -0.997500)
        ha = ACC;
    else if(ha == CON && state.vel >= 0.000000 && r.DistTraveled(r.vMax, r.decMax) - state.pos >= -128.399994)
        ha = CON;

    return ha;
}

/*
 * This is an action-selection policy generated by LDIPS, with error
 */
HA ASP_LDIPS_error(HA ha, Obs state, Robot& r){
    // Copy paste below
    if(ha == CON && r.DistTraveled(state.vel, r.decMax) - r.DistTraveled(r.vMax, r.decMax) >= 9.714069)
        ha = CON;
    else if(ha == DEC && r.DistTraveled(state.vel, r.decMax) - r.target >= 11.458965)
        ha = CON;
    else if(ha == CON && state.pos + state.pos + state.pos - r.target >= 35.615170)
        ha = DEC;
    else if(ha == ACC && r.DistTraveled(state.vel, r.decMax) + r.target >= 535.545532)
        ha = CON;
    else if(ha == CON)
        ha = ACC;
    else if(ha == ACC && state.pos - r.target + r.DistTraveled(state.vel, r.decMax) >= -0.138184)
        ha = DEC;
    else if(ha == DEC && r.DistTraveled(state.vel, r.decMax) - state.pos - state.pos >= -49.242615)
        ha = ACC;
    else if(ha == DEC)
        ha = DEC;
    else if(ha == ACC)
        ha = ACC;

    return ha;
}

/*
 * This is a simplistic action-selection policy with only ACC and DEC
 */
HA ASP_accDecOnly(HA ha, Obs state, Robot& r){
    if(state.pos < r.target / 2){
        ha = ACC;
    } else {
        ha = DEC;
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
                        &ASP_LDIPS,             // 1
                        &ASP_LDIPS_error,       // 2
                        &ASP_Hand_prob,         // 3
                        &ASP_accDecOnly,        // 4
                        &ASP_random,            // 5
                        &ASP_Sim,               // 6
                        &ASP_Hand_clean,         // 7
                        &ASP_consistent_prob    // 8
                        };

asp_t* ASP_model(int model){
    return ASPs[model];
}