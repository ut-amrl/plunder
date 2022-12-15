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
string to_string(HA ha){
    if(ha == ACC) return "ACC";
    if(ha == DEC) return "DEC";
    if(ha == CON) return "CON";
    return "CON";
}

HA to_label(string str){
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

class Robot;

struct Trajectory {
    int T;
    Robot& r;
    vector<State> traj;

    Trajectory(Robot& robot) : T(0), r(robot) {}

    void append(State s) {
        traj.push_back(s);
        T++;
    }

    State get(int t) {
        return traj[t];
    }

    // void append(HA ha){
    //     traj.push_back(State { ha });
    //     T++;
    // }

    // void append(LA la){
    //     traj.push_back(State { HA{}, la });
    //     T++;
    // }

    // void append(Obs obs){
    //     traj.push_back(State { HA{}, LA{}, obs});
    //     T++;
    // }
};