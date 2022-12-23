#pragma once

#include "settings.h"

using namespace std;

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

// TODO (major overhaul): convert these all to a map<string, double>, abstracting them away

struct LA { // Low-level actions
    double acc; // acceleration
};

struct Obs { // Observations
    double pos; // position
    double vel; // velocity
};