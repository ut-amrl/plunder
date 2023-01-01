#pragma once

#include "settings.h"

using namespace std;
using namespace AST;

// -----------------------------------------------------------------------------
// ----- Problem domain --------------------------------------------------------
// -----------------------------------------------------------------------------

// ----- User-defined ---------------------------------------------
// HA: High-level action labels. Make sure the *first* label is the desired initial high-level action!
enum HA_enum {
    FASTER,
    SLOWER,
    LANE_LEFT,
    LANE_RIGHT
};
vector<string> HA_Labels = {
    "FASTER",
    "SLOWER",
    "LANE_LEFT",
    "LANE_RIGHT"
};

// Optional: create safe transitions. To turn this feature on, toggle USE_SAFE_TRANSITIONS in settings.
map<HA, vector<HA>> valid_transitions = {};

// LA: Defines the low-level action of a robot. (Variables manipulated by the motor model)
vector<string> LA_vars = {
    "steer",
    "acc"
};

// Obs: Defines the world state of a robot. 
vector<Var> Obs_vars = {
    Var ("l_x", Dimension(0, 0, 0), true),
    Var ("l_vx", Dimension(0, 0, 0), true),
    Var ("l_vy", Dimension(0, 0, 0), true),
    Var ("f_x", Dimension(0, 0, 0), true),
    Var ("f_vx", Dimension(0, 0, 0), true),
    Var ("f_vy", Dimension(0, 0, 0), true),
    Var ("r_x", Dimension(0, 0, 0), true),
    Var ("r_vx", Dimension(0, 0, 0), true),
    Var ("r_vy", Dimension(0, 0, 0), true),
};