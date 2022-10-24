#pragma once

#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>

#include "settings.h"

#define PRECISION 10
#define robotEpsilon 10E-10
#define SEED 0 // Random seed

using namespace std;

// ----- Markov System definitions ---------------------------------------------

enum HA { // High-level actions
    ACC, // Constant acceleration
    DEC, // Constant deceleration
    CON  // No acceleration
};

const uint numHA = 3;

struct LA { // Low-level actions
    double acc; // target acceleration
};

struct Obs { // State observations
    double pos; // position
    double vel; // velocity
};

// Random error distributions
random_device rd;
default_random_engine gen(SEED);

// Helper functions
string HAToString(HA ha){
    if(ha == ACC) return "ACC";
    if(ha == DEC) return "DEC";
    if(ha == CON) return "CON";
    return "CON";
}

HA stringToHA(string str){
    if(str == "ACC") return ACC;
    if(str == "DEC") return DEC;
    if(str == "CON") return CON;
    return CON;
}


// ----- Robot Class ---------------------------------------------

// class 

class Robot {

    private:
    public:

    // Constant inputs into LDIPS
    double accMax; // Maximum constant acceleration
    double decMax; // Maximum constant deceleration 
    double vMax;   // Maximum velocity
    double target; // Target distance


    normal_distribution<double> accErrDistr;      // Acceleration error distribution
    double pointAccuracy;                         // Probability of transitioning to the correct high-level action

    Robot(double _accMax, double _decMax, double _vMax, double _target, normal_distribution<double> _accErrDistr, double _pointAccuracy){
        accMax = _accMax;
        decMax = _decMax;
        vMax = _vMax;
        target = _target;
        accErrDistr = _accErrDistr;
        pointAccuracy = _pointAccuracy;
    }

    Robot() {}

    HA ha = ACC;                        // Initial high-level action
    LA la = { .acc = 0 };               // Initial low-level action
    Obs state = { .pos = 0, .vel = 0 }; // Initial observed state

    // MOTION MODEL (ACTION-SELECTION POLICY): Transition to new high-level action based on current action and state
    void runASP(HA (*ASP) (HA, Obs, Robot&)){
        ha = ASP(ha, state, *this);
    }

    // MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
    LA motorModel(HA ha, Obs state, bool error){
        double acc = 0;
        
        if(ha == ACC){
            // acc = accMax;
            acc = (vMax - state.vel) / vMax * accMax + activationMinAcc;
        } else if (ha == DEC) {
            acc = decMax;
            // acc = (vMax - state.vel) / vMax * decMax - activationMinAcc;
        }

        // Induce some error
        if(error){
            acc += accErrDistr(gen);
        }

        return LA { .acc = acc };
    }

    // PHYSICS SIM: Given a current high-level action, apply a motor controller and update observed state. Runs once per time step
    void updatePhysics(double t_step){
        double vPrev = state.vel;
        double xPrev = state.pos;
        
        // Update velocity and displacement accordingly
        state.vel = vPrev + la.acc * t_step;

        if(state.vel < robotEpsilon){ // Round to 0
            state.vel = 0;
        }

        if(abs(state.vel - vMax) < robotEpsilon){ // Round to vMax
            state.vel = vMax;
        }

        if(abs(state.pos - target) < robotEpsilon){ // Round to target
            state.pos = target;
        }

        state.pos = xPrev + (state.vel + vPrev)/2 * t_step;
    }

    void updateLA(){
        // Select some action (acceleration)
        la = motorModel(ha, state, true);
    }

    // HELPER METHODS

    // Return the distance this robot would travel before stopping if it began decelerating immediately
    double DistTraveled(double v, double dec){
        return - v * v / (2 * dec);
        // double ln = log(((decMax - activationMinAcc) * vMax) / ((decMax - activationMinAcc) * vMax - decMax * state.vel));
        // double num = vMax * ((decMax - activationMinAcc) * vMax * ln - decMax * state.vel);
        // return - num / (decMax * decMax);
    }

    // Seeded random generator
    bool sampleDiscrete(double probTrue){
        double rv = ((double) rand())/RAND_MAX;
        return rv <= probTrue;
    }

    // Robot has reached target and is at rest. End simulation.
    bool finished(){
        return state.vel < robotEpsilon && state.pos >= target - robotEpsilon;
    }

    // Reset robot
    void reset(){
        ha = ACC;
        la = LA { .acc = 0 };
        state = Obs { .pos = 0, .vel = 0 };
    }
};