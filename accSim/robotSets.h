#pragma once

#include <stdlib.h>
#include <vector>

#include "../robot.h"

using namespace std;

// ----- Robot Test Sets ---------------------------------------------

vector<Robot> getRobotSet(int setNum, normal_distribution<double> accErrDistr, double haProbCorrect){
    if(setNum == 0){
        return {
            Robot(6, -5, 10, 150, accErrDistr, haProbCorrect),
            Robot(3, -3, 20, 100, accErrDistr, haProbCorrect),
            Robot(5, -2, 8, 200, accErrDistr, haProbCorrect),
            Robot(8, -3, 20, 50, accErrDistr, haProbCorrect),
            Robot(1.5, -2, 2, 30, accErrDistr, haProbCorrect),
            Robot(3, -2, 3, 20, accErrDistr, haProbCorrect),
            Robot(9, -2, 2, 15, accErrDistr, haProbCorrect),
            Robot(1.5, -2, 30, 300, accErrDistr, haProbCorrect),
            Robot(2, -2, 80, 300, accErrDistr, haProbCorrect),
            Robot(5, -2, 15, 200, accErrDistr, haProbCorrect),
            Robot(10, -10, 20, 500, accErrDistr, haProbCorrect),
            Robot(25, -1, 15, 400, accErrDistr, haProbCorrect),
            Robot(300, -300, 500, 20000, accErrDistr, haProbCorrect),
            Robot(1000, -500, 3000, 50000, accErrDistr, haProbCorrect),
            Robot(8000, -6000, 30000, 400000, accErrDistr, haProbCorrect)
        };
    } else if(setNum == 1){
        return {
            Robot(5, -2, 15, 80, accErrDistr, haProbCorrect),
            Robot(3, -3, 4, 80, accErrDistr, haProbCorrect),
            Robot(2, -4, 50, 80, accErrDistr, haProbCorrect),
            Robot(8, -6, 20, 80, accErrDistr, haProbCorrect),
            Robot(4, -5, 100, 80, accErrDistr, haProbCorrect)
        };
    } else {
        return {
            Robot(6, -5, 10, 150, accErrDistr, haProbCorrect),
            Robot(3, -3, 20, 100, accErrDistr, haProbCorrect),
            Robot(5, -2, 8, 200, accErrDistr, haProbCorrect),
            Robot(8, -3, 20, 50, accErrDistr, haProbCorrect),
            Robot(1.5, -2, 2, 30, accErrDistr, haProbCorrect),
        };
    }
}
