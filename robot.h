#pragma once

#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>
#include <algorithm>

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

struct Obs { // Observations
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
        reset();
    }

    Robot(const Robot& other) : accMax(other.accMax), decMax(other.decMax), vMax(other.vMax), target(other.target),
                                accErrDistr(other.accErrDistr), pointAccuracy(other.pointAccuracy),
                                ha(other.ha), la(other.la), obs(other.obs) {

    }

    Robot() {}

    HA ha;
    LA la;
    Obs obs;

    // MOTION MODEL (ACTION-SELECTION POLICY): Transition to new high-level action based on current action and state
    void runASP(HA (*ASP) (HA, Obs, Robot&)){
        ha = ASP(ha, obs, *this);
    }

    // MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
    LA motorModel(HA ha, Obs obs, LA prevLa, bool error){
        double change = laChangeSpeed;
        LA newLa;
        if(error){
            change += accErrDistr(gen) * (switchingError / stddevError);
        }
        
        if(ha == ACC){
            newLa.acc = min(prevLa.acc + change, accMax);
        } else if (ha == DEC) {
            newLa.acc = max(prevLa.acc - change, decMax);
        } else {
            if(prevLa.acc < 0)
                newLa.acc = min(0.0, prevLa.acc + change);
            if(prevLa.acc > 0)
                newLa.acc = max(0.0, prevLa.acc - change);
        }

        // Induce some additional lesser error
        if(error){
            newLa.acc += accErrDistr(gen);
        }

        return newLa;
    }

    // PHYSICS SIM: Given a current high-level action, apply a motor controller and update observed state. Runs once per time step
    void updatePhysics(double t_step){
        double vPrev = obs.vel;
        double xPrev = obs.pos;
        
        // Update velocity and displacement accordingly
        obs.vel = vPrev + la.acc * t_step;

        if(obs.vel < robotEpsilon){ // Round to 0
            obs.vel = 0;
        }

        if(abs(obs.vel - vMax) < robotEpsilon){ // Round to vMax
            obs.vel = vMax;
        }

        if(abs(obs.pos - target) < robotEpsilon){ // Round to target
            obs.pos = target;
        }

        obs.pos = xPrev + (obs.vel + vPrev)/2 * t_step;
    }

    void updateLA(){
        // Select some action (acceleration)
        la = motorModel(ha, obs, la, true);
    }

    // HELPER METHODS

    // Return the distance this robot would travel before stopping if it began decelerating immediately
    double DistTraveled(double v, double dec){
        return - v * v / (2 * dec);
    }

    // Seeded random generator
    bool sampleDiscrete(double probTrue){
        double rv = ((double) rand())/RAND_MAX;
        return rv <= probTrue;
    }

    // Reset robot
    void reset(){
        ha = ACC;
        la = LA { .acc = accMax };
        obs = Obs { .pos = 0, .vel = 0 };
    }
};