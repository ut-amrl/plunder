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
    MOVE_TO_CUBE_BOTTOM,
    MOVE_TO_TARGET,
    LIFT,
    MOVE_TO_CUBE_TOP,
    GRASP
};
vector<string> HA_Labels = {
    "MOVE_TO_CUBE_BOTTOM",
    "MOVE_TO_TARGET",
    "LIFT",
    "MOVE_TO_CUBE_TOP",
    "GRASP"
};

// Optional: create safe transitions. To turn this feature on, toggle USE_SAFE_TRANSITIONS in settings.
map<HA, vector<HA>> valid_transitions = {};

// LA: Defines the low-level action of a robot. (Variables manipulated by the motor model)
vector<string> LA_vars = {
    "vx",
    "vy",
    "vz",
    "end"
};

// Obs: Defines the world state of a robot. Units: [m, t, deg]
vector<Var> Obs_vars = {
    Var ("x", Dimension(1, 0, 0), true),
    Var ("y", Dimension(1, 0, 0), true),
    Var ("z", Dimension(1, 0, 0), true),
    Var ("end_width", Dimension(1, 0, 0), false),
    Var ("bx1", Dimension(1, 0, 0), true),
    Var ("by1", Dimension(1, 0, 0), true),
    Var ("bz1", Dimension(1, 0, 0), true),
    Var ("bx2", Dimension(1, 0, 0), true),
    Var ("by2", Dimension(1, 0, 0), true),
    Var ("bz2", Dimension(1, 0, 0), true),
    Var ("tx1", Dimension(1, 0, 0), false),
    Var ("ty1", Dimension(1, 0, 0), false),
    Var ("tz1", Dimension(1, 0, 0), false),
    Var ("tx2", Dimension(1, 0, 0), true),
    Var ("ty2", Dimension(1, 0, 0), true),
    Var ("tz2", Dimension(1, 0, 0), true),
    Var ("vx", Dimension(1, -1, 0), false),
    Var ("vy", Dimension(1, -1, 0), false),
    Var ("vz", Dimension(1, -1, 0), false),
    Var ("end", Dimension(1, -1, 0), false),
};