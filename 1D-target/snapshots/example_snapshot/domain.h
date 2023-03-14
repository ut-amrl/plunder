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
    ACC,
    CON,
    DEC
};
vector<string> HA_Labels = {
    "ACC", // Accelerate
    "CON", // Constant velocity
    "DEC"  // Decelerate
};

// Optional: create safe transitions. To turn this feature on, toggle USE_SAFE_TRANSITIONS in settings.
map<HA, vector<HA>> valid_transitions = {
    { ACC, {ACC, DEC, CON} },
    { CON, {DEC, CON} },
    { DEC, {DEC} }
};

// LA: Defines the low-level action of a robot. (Variables manipulated by the motor model)
vector<string> LA_vars = {
    "acc"   // acceleration
};

// Obs_vars: Defines the world state of a robot.  Dimensions are (m, s, deg)
// Structure: 
//      Var ( "dns", Dimension(1, 0, 0), true ) indicates a variable named "dns" with units (m), that will be used in the synthesis step.
//      Var ( "acc", Dimension(1, -2, 0), false ) indicates a variable named "acc" with units (m/s^2), that will not be used in the synthesis step.
vector<Var> Obs_vars = {
    Var ("acc", Dimension(1, -2, 0), false),
    Var ("pos", Dimension(1, 0, 0), false), // We will use dns instead of pos
    Var ("decMax", Dimension(1, -2, 0), true),
    Var ("accMax", Dimension(1, -2, 0), true),
    Var ("vMax", Dimension(1, -1, 0), true),
    Var ("vel", Dimension(1, -1, 0), true),
    Var ("target", Dimension(1, 0, 0), true),
    Var ("dns", Dimension(1, 0, 0), true),
};