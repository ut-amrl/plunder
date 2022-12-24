#pragma once

#include "settings.h"

using namespace std;
typedef int HA;

// -----------------------------------------------------------------------------
// ----- Problem domain --------------------------------------------------------
// -----------------------------------------------------------------------------

// ----- User-defined ---------------------------------------------
enum HA_enum {
    ACC,
    DEC,
    CON
};
vector<string> HA_Labels = { // High-level action labels
    "ACC", // Constant acceleration
    "DEC", // Constant deceleration
    "CON"  // No acceleration
};

// Define valid transitions
vector<HA> valid_transitions(HA ha, bool use_safe_transitions){
    vector<HA> valid;
    if(use_safe_transitions){
        if(ha == ACC) valid={ACC, CON, DEC};
        if(ha == CON) valid={CON, DEC};
        if(ha == DEC) valid={DEC};
    } else {
        valid={ACC, DEC, CON};
    }
    return valid;
}

// TODO (major overhaul): convert these all to a map<string, double>, abstracting them away

struct LA { // Low-level actions
    double acc; // acceleration
};

struct Obs { // Observations
    double pos; // position
    double vel; // velocity
};