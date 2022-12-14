#pragma once

#include "settings.h"

using namespace std;

// -----------------------------------------------------------------------------
// ----- Problem domain --------------------------------------------------------
// -----------------------------------------------------------------------------

// ----- User-defined ---------------------------------------------

enum HA { // High-level labels
    ACC, // Constant acceleration
    DEC, // Constant deceleration
    CON  // No acceleration
};

const uint numHA = 3;

struct LA { // Low-level actions
    double acc; // acceleration
};

struct Obs { // Observations
    double pos; // position
    double vel; // velocity
};

// Helper functions
string HAToString(HA ha){
    if(ha == ACC) return "ACC";
    if(ha == DEC) return "DEC";
    if(ha == CON) return "CON";
    return "CON";
}

HA stringToHA(string str){
    if(str == "ACC") return ACC;
    if(str == "DEC") return DEC;
    if(str == "CON") return CON;
    return CON;
}


// ----- Markov System Definitions --------------------------------
struct State {
    HA ha;
    LA la;
    Obs obs;
};

