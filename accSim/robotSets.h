#pragma once

#include <stdlib.h>
#include <vector>

#include "../robot.h"

using namespace std;

// ----- Robot Test Sets ---------------------------------------------

vector<Robot> getRobotSet(int setNum, normal_distribution<double> accErrDistr, double haProbCorrect){
    if(setNum == 0){
        return {
            Robot(6, -5, 15, 150, accErrDistr, haProbCorrect),
            Robot(3, -3, 30, 100, accErrDistr, haProbCorrect),
            Robot(5, -2, 12, 200, accErrDistr, haProbCorrect),
            Robot(8, -3, 30, 50, accErrDistr, haProbCorrect),
            Robot(1.5, -2, 3, 30, accErrDistr, haProbCorrect),
            Robot(3, -2, 4, 20, accErrDistr, haProbCorrect),
            Robot(0.5, -1, 2, 15, accErrDistr, haProbCorrect),
            Robot(1.5, -2, 40, 300, accErrDistr, haProbCorrect),
            Robot(2, -2, 100, 300, accErrDistr, haProbCorrect),
            Robot(5, -1, 25, 200, accErrDistr, haProbCorrect),
            Robot(10, -10, 25, 500, accErrDistr, haProbCorrect)
        };
    } else if(setNum == 1){
        return {
            Robot(5, -2, 15, 80, accErrDistr, haProbCorrect),
            Robot(3, -3, 4, 80, accErrDistr, haProbCorrect),
            Robot(1.5, -4, 50, 80, accErrDistr, haProbCorrect),
            Robot(8, -6, 20, 80, accErrDistr, haProbCorrect),
            Robot(4, -5, 100, 80, accErrDistr, haProbCorrect)
        };
    } else {
        return {
            Robot(5, -4, 12, 100, accErrDistr, haProbCorrect)
        };
    }
}