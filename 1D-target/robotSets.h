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
        // Training set
        make_state(6, -5, 10, 100),
        make_state(15, -3, 40, 100),
        make_state(8, -9, 20, 100),
        make_state(9, -6, 12, 100),
        make_state(7, -10, 25, 100),
        make_state(5, -7, 30, 100),
        make_state(6, -5, 30, 100),
        make_state(12, -10, 25, 100),
        make_state(13, -20, 40, 100),
        make_state(5, -8, 15, 100),

        // Validation set
        make_state(10, -5, 20, 100),
        make_state(5, -8, 10, 100),
        make_state(4, -4, 30, 100),
        make_state(4, -4, 5, 100),
        make_state(10, -10, 50, 100),
        make_state(30, -30, 200, 100),
        make_state(30, -24, 11, 100),
        make_state(8, -27, 38, 100),
        make_state(27, -16, 11, 100),
        make_state(10, -21, 28, 100),
        make_state(28, -28, 23, 100),
        make_state(16, -9, 81, 100),
        make_state(19, -26, 67, 100),
        make_state(10, -17, 7, 100),
        make_state(19, -25, 27, 100),
        make_state(20, -8, 10, 100),
        make_state(27, -20, 72, 100),
        make_state(21, -7, 99, 100),
        make_state(19, -12, 12, 100),
        make_state(10, -12, 10, 100)
    };
}
