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
    MOVE_TO_CUBE,
    MOVE_TO_TARGET
};
vector<string> HA_Labels = {
    "MOVE_TO_CUBE",
    "MOVE_TO_TARGET"
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

// Obs: Defines the world state of a robot. Units: [m, s, radians]
vector<Var> Obs_vars = {
    Var ("x", Dimension(1, 0, 0), true),
    Var ("y", Dimension(1, 0, 0), true),
    Var ("z", Dimension(1, 0, 0), true),
    Var ("bx", Dimension(1, 0, 0), true),
    Var ("by", Dimension(1, 0, 0), true),
    Var ("bz", Dimension(1, 0, 0), true),
    Var ("tx", Dimension(1, 0, 0), true),
    Var ("ty", Dimension(1, 0, 0), true),
    Var ("tz", Dimension(1, 0, 0), true),
    Var ("end_width", Dimension(1, 0, 0), false),
    Var ("vx", Dimension(1, -2, 0), false),
    Var ("vy", Dimension(1, -2, 0), false),
    Var ("vz", Dimension(1, -2, 0), false),
    Var ("end", Dimension(1, -2, 0), false),
};