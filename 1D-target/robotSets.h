#pragma once

#include "robot.h"

using namespace std;

// ----- State Test Sets ---------------------------------------------
State make_state(double _accMax, double _decMax, double _vMax, double _target) {
    map<string, double> table = {
        { "accMax", _accMax },
        { "decMax", _decMax },
        { "vMax", _vMax },
        { "target", _target }
    };

    return State (table);
}

vector<State> getInitStates(){
    return {
        make_state(6, -5, 12, 100),
        make_state(3, -4, 20, 100),
        make_state(5, -3, 8, 100),
        make_state(8, -4, 20, 100),
        make_state(5, -3, 12, 100),
        make_state(3, -3, 15, 100),
        make_state(9, -6, 12, 100),
        make_state(4, -3, 30, 100),
        make_state(7, -7, 200, 100),
        make_state(5, -4, 15, 100),
        make_state(10, -10, 20, 100),
        make_state(30, -30, 50, 100),
    };

    // return {
    //     make_state(6, -5, 12, 150),
    //     make_state(3, -4, 20, 100),
    //     make_state(5, -3, 8, 200),
    //     make_state(8, -4, 20, 50),
    //     make_state(5, -3, 12, 30),
    //     make_state(3, -3, 15, 20),
    //     make_state(9, -6, 12, 15),
    //     make_state(4, -3, 30, 300),
    //     make_state(7, -7, 200, 300),
    //     make_state(5, -4, 15, 200),
    //     make_state(10, -10, 20, 500),
    // };

    // return {
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    //     make_state(4, -3, 15, 100),
    //     make_state(4, -3, 15, 100),
    //     make_state(4, -3, 15, 100),
    //     make_state(4, -3, 15, 100),
    //     make_state(6, -5, 15, 50),
    //     make_state(6, -5, 15, 50),
    //     make_state(6, -5, 15, 50),
    //     make_state(6, -5, 15, 50),
    // };

    // return {
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    //     make_state(5, -6, 15, 150),
    // };
}
