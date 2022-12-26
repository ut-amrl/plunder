#pragma once

#include "settings.h"
#include "ast/ast.hpp"
#include "ast/enumeration.hpp"
#include "ast/library_functions.hpp"
#include "ast/parsing.hpp"
#include "visitors/interp_visitor.hpp"
#include "ast/synthesis.hpp"

using namespace std;
using namespace AST;
typedef int HA;

// -----------------------------------------------------------------------------
// ----- Problem domain --------------------------------------------------------
// -----------------------------------------------------------------------------

// ----- User-defined ---------------------------------------------
// HA: High-level action labels
enum HA_enum {
    ACC,
    DEC,
    CON
};
vector<string> HA_Labels = {
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

// LA: Defines the low-level action of a robot. (Variables manipulated by the motor model)
vector<string> LA_vars = {
    "acc"
};

// Obs: Defines the world state of a robot. 
// Structure: 
//      Var ( "pos", Dimension(1, 0, 0), true ) indicates a variable named "pos" with units (m), that will be used in the synthesis step.
//      Var ( "acc", Dimension(1, -2, 0), false ) indicates a variable named "accMax" with units (m/s^2), that will not be used in the synthesis step.
vector<Var> Obs_vars = {
    Var ("pos", Dimension(1, 0, 0), true),
    Var ("vel", Dimension(1, -1, 0), true),
    Var ("acc", Dimension(1, -2, 0), false),
    Var ("accMax", Dimension(1, -2, 0), false),
    Var ("decMax", Dimension(1, -2, 0), true),
    Var ("vMax", Dimension(1, -1, 0), true),
    Var ("target", Dimension(1, 0, 0), true)
};