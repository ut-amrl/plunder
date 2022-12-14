#pragma once

#include "robot.h"

using namespace std;

// ----- Robot Test Sets ---------------------------------------------

vector<Robot> getRobotSet(int setNum){
    if(setNum == 0){ // Varied assortment of robots
        return {
            Robot(6, -5, 12, 150),
            Robot(3, -4, 20, 100),
            Robot(5, -3, 8, 200),
            Robot(8, -4, 20, 50),
            Robot(5, -3, 12, 30),
            Robot(3, -3, 15, 20),
            Robot(9, -6, 12, 15),
            Robot(4, -3, 30, 300),
            Robot(7, -7, 200, 300),
            Robot(5, -4, 15, 200),
            Robot(10, -10, 20, 500),
        };
    } else if(setNum == 1){ // Repeated demonstrations
        return {
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
            Robot(4, -3, 15, 100),
            Robot(4, -3, 15, 100),
            Robot(4, -3, 15, 100),
            Robot(4, -3, 15, 100),
            Robot(6, -5, 15, 50),
            Robot(6, -5, 15, 50),
            Robot(6, -5, 15, 50),
            Robot(6, -5, 15, 50),
        };
    } else {
        return {
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
            Robot(5, -6, 15, 150),
        };
    }
}
