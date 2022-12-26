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

// Optional: create safe transitions. To turn this feature on, toggle USE_SAFE_TRANSITIONS in settings.
map<HA, vector<HA>> valid_transitions = {
    { ACC, {ACC, DEC, CON} },
    { CON, {DEC, CON} },
    { DEC, {DEC} }
};

// TODO (major overhaul): convert these all to a map<string, double>, abstracting them away

string LA = "acc"; // TODO

struct Obs { // Observations
    double pos; // position
    double vel; // velocity
    double acc; // acceleration
};