#pragma once

#include "domain.h"
#include "system.h"

using namespace std;
using namespace SETTINGS;

// TODO
// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
map<string, normal_distribution<double>> la_error = {
    { "steer", normal_distribution<double>(0.0, 0.1) },
    { "acc", normal_distribution<double>(0.0, 0.1) }
};

Obs motorModel(State state, bool error){
    return state.obs;
}


HA ASP_model(State state){}
Obs physicsModel(State state, double t_step){}