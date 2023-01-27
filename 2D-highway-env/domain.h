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
map<HA, vector<HA>> valid_transitions = {
    { FASTER, {FASTER, SLOWER, LANE_LEFT, LANE_RIGHT} },
    { SLOWER, {FASTER, SLOWER, LANE_LEFT, LANE_RIGHT} },
    { LANE_LEFT, {FASTER, SLOWER, LANE_LEFT} },
    { LANE_RIGHT, {FASTER, SLOWER, LANE_RIGHT} }
};

// LA: Defines the low-level action of a robot. (Variables manipulated by the motor model)
vector<string> LA_vars = {
    "steer",
    "acc"
};

// Obs: Defines the world state of a robot. Units: [m, s, radians]
vector<Var> Obs_vars = {
    Var ("x", Dimension(1, 0, 0), true),
    Var ("y", Dimension(1, 0, 0), true),
    Var ("vx", Dimension(1, -1, 0), false),
    Var ("heading", Dimension(0, 0, 1), false),
    Var ("l_x", Dimension(1, 0, 0), true),
    Var ("l_vx", Dimension(1, -1, 0), false),
    Var ("l_heading", Dimension(0, 0, 1), false),
    Var ("f_x", Dimension(1, 0, 0), true),
    Var ("f_vx", Dimension(1, -1, 0), false),
    Var ("f_heading", Dimension(0, 0, 1), false),
    Var ("r_x", Dimension(1, 0, 0), true),
    Var ("r_vx", Dimension(1, -1, 0), false),
    Var ("r_heading", Dimension(0, 0, 1), false),
    Var ("steer", Dimension(0, -2, 1), false),
    Var ("acc", Dimension(1, -2, 0), false)
};