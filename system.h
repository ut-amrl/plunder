#pragma once

#include "utils.h"

using namespace std;
using namespace SETTINGS;

// ----- Markov System Definitions --------------------------------

struct State {
    HA ha;
    Obs obs;
};

class Robot;

typedef HA asp(State, Robot&);
typedef Obs motor(State, Robot&, bool);
typedef Obs phys(State, Robot&, double);

struct Trajectory {
    Robot& r;
    vector<State> traj;

    Trajectory(Robot& robot) : r(robot) {}

    void append(State s) {
        traj.push_back(s);
    }

    State get(int t) {
        return traj[t];
    }

    void set(int t, State s){
        traj[t] = s;
    }

    int size() {
        return traj.size();
    }
};
