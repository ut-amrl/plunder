#pragma once

#include <stdlib.h>
#include <vector>

#include "robot.h"

using namespace std;

// ----- Robot Test Sets ---------------------------------------------

vector<Robot> getRobotSet(int setNum, normal_distribution<double> accErrDistr, double haProbCorrect){
    if(setNum == 0){ // Varied assortment of robots
        return {
            Robot(6, -5, 12, 150, accErrDistr, haProbCorrect),
            Robot(3, -4, 20, 100, accErrDistr, haProbCorrect),
            Robot(5, -3, 8, 200, accErrDistr, haProbCorrect),
            Robot(8, -4, 20, 50, accErrDistr, haProbCorrect),
            Robot(5, -3, 12, 30, accErrDistr, haProbCorrect),
            Robot(3, -3, 15, 20, accErrDistr, haProbCorrect),
            Robot(9, -6, 12, 15, accErrDistr, haProbCorrect),
            Robot(4, -3, 30, 300, accErrDistr, haProbCorrect),
            Robot(7, -7, 200, 300, accErrDistr, haProbCorrect),
            Robot(5, -4, 15, 200, accErrDistr, haProbCorrect),
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
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
            Robot(5, -6, 15, 150, accErrDistr, haProbCorrect),
        };
    }
}
