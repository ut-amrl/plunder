#pragma once

#include <stdlib.h>
#include <vector>

#include "robot.h"

using namespace std;

// ----- Robot Test Sets ---------------------------------------------

vector<Robot> getRobotSet(int setNum, normal_distribution<double> accErrDistr, double haProbCorrect){
    if(setNum == 0){ // Varied assortment of robots
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
        };
    } else if(setNum == 1){ // Repeated demonstrations
        return {
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(4, -3, 15, 100, accErrDistr, haProbCorrect),
            Robot(4, -3, 15, 100, accErrDistr, haProbCorrect),
            Robot(4, -3, 15, 100, accErrDistr, haProbCorrect),
            Robot(4, -3, 15, 100, accErrDistr, haProbCorrect),
            Robot(6, -5, 15, 50, accErrDistr, haProbCorrect),
            Robot(6, -5, 15, 50, accErrDistr, haProbCorrect),
            Robot(6, -5, 15, 50, accErrDistr, haProbCorrect),
            Robot(6, -5, 15, 50, accErrDistr, haProbCorrect),
        };
    } else {
        return {
            Robot(6, -5, 10, 150, accErrDistr, haProbCorrect),
            Robot(3, -3, 20, 100, accErrDistr, haProbCorrect),
            Robot(5, -2, 8, 200, accErrDistr, haProbCorrect),
            Robot(3, -8, 20, 50, accErrDistr, haProbCorrect),
            Robot(10, -8, 15, 300, accErrDistr, haProbCorrect),
        };
    }
}
