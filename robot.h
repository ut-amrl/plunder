#pragma once

#include "domain.h"
#include "utils.h"

using namespace std;
using namespace SETTINGS;

// -----------------------------------------------------------------------------
// ----- Robot Class--- --------------------------------------------------------
// -----------------------------------------------------------------------------
class Robot;

typedef HA asp(State, Robot&);
typedef LA motor(State, Robot&, bool);
typedef Obs phys(State, Robot&, double);

class Robot {
public:

    // ------- Robot parameters -----------
    double accMax; // Maximum constant acceleration
    double decMax; // Maximum constant deceleration 
    double vMax;   // Maximum velocity
    double target; // Target distance

    State state;

    Robot(double _accMax, double _decMax, double _vMax, double _target) : accMax(_accMax), decMax(_decMax), vMax(_vMax), target(_target) {
        reset();
    }

    Robot() {}

    void runASP(asp*);
    void updateLA(bool, motor*);
    void updateObs(phys*, double);

    // Reset robot
    void reset(){
        state = {};
    }
};


// ACTION-SELECTION POLICY: Transition to new high-level action based on current action and state
// ----- Curated Selection of ASPs ---------------------------------------------

/*
* This is a hand-crafted, non-probabilistic action-selection policy.
*/
HA ASP_Hand(State state, Robot& r){
    Obs obs = state.obs;
    HA ha = state.ha;

    double xToTarget = r.target - obs.pos;                                  // distance to the target

    bool cond1 = obs.vel - r.vMax >= 0;                                     // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget - DistTraveled(obs.vel, r.decMax) < EPSILON;  // needs to decelerate or else it will pass target

    if(ha == ACC){                          // transitions starting with ACC
        if(cond1 && !cond2) ha=CON;         // ACC -> CON (expected)
        else if(cond2) ha=DEC;              // ACC -> DEC (expected)
        else ha=ACC;                           // "no transition" is the default
    }
    else if(ha == CON){                          // transitions starting with CON
        // if(!cond1 && !cond2) ha=ACC;        // CON -> ACC (rare)
        if(false) ha=ACC;
        else if(cond2) ha=DEC;              // CON -> DEC (expected)
        else ha=CON;                           // "no transition" is the default
    }
    else if(ha == DEC){
        // if(!cond1 && !cond2) ha=ACC;        // DEC -> ACC (0)
        // else if(cond1 && !cond2) ha=CON;    // DEC -> CON (0)
        if(false) ha=ACC;
        else if(false) ha=CON;
        else ha=DEC;                           // "no transition" is the default
    }

    return ha;
}

/*
* This is a probabilistic hand-crafted action-selection policy.
*/
HA ASP_Hand_prob(State state, Robot& r){
    Obs obs = state.obs;
    HA ha = state.ha;

    double xToTarget = r.target - obs.pos;                            // distance to the target

    bool cond1 = obs.vel - r.vMax >= 0;                                     // is at max velocity (can no longer accelerate)
    bool cond2 = xToTarget - DistTraveled(obs.vel, r.decMax) < EPSILON;  // needs to decelerate or else it will pass target

    bool cond1smooth = flip(logistic(0, 2.5, obs.vel - r.vMax));
    bool cond2smooth = flip(logistic(0, -1, xToTarget - DistTraveled(obs.vel, r.decMax)));

    if(ha == ACC){                          // transitions starting with ACC
        if(cond1smooth && !cond2smooth) ha=CON;         // ACC -> CON (expected)
        else if(cond2smooth) ha=DEC;              // ACC -> DEC (expected)
        else ha=ACC;                           // "no transition" is the default
    }
    else if(ha == CON){                          // transitions starting with CON
        // if(!cond1smooth && !cond2smooth) ha=ACC;        // CON -> ACC (rare)
        if(false) ha=ACC;        // CON -> ACC (rare)
        else if(cond2smooth) ha=DEC;              // CON -> DEC (expected)
        else ha=CON;                           // "no transition" is the default
    }
    else if(ha == DEC){
        // if(!cond1 && !cond2) ha=ACC;        // DEC -> ACC (0)
        // else if(cond1 && !cond2) ha=CON;    // DEC -> CON (0)
        if(false) ha=ACC;        // DEC -> ACC (0)
        else if(false) ha=CON;    // DEC -> CON (0)
        else ha=DEC;                           // "no transition" is the default
    }

    return ha;
}

/*
* This is a uniformly random ASP.
*/
HA ASP_random(State state, Robot& r){
    int mod = rand() % numHA;
    HA ha = to_label(mod);

    return ha;
}

// Select an ASP to use
vector<asp*> ASPs = { &ASP_Hand,              // 0
                      &ASP_Hand_prob,         // 1
                      &ASP_random,            // 2
                    };

asp* ASP_model(int model){
    return ASPs[model];
}

// MOTOR (OBSERVATION) MODEL: known function mapping from high-level to low-level actions
normal_distribution<double> la_error = normal_distribution<double>(MEAN_ERROR, STDDEV_ERROR);
LA motorModel(State state, Robot& r, bool error){
    HA ha = state.ha; LA la = state.la; Obs obs = state.obs;

    double change = JERK;
    if(error){
        change += la_error(gen) * (JERK_ERROR / STDDEV_ERROR);
    }
    
    if(ha == ACC){
        la.acc = min(la.acc + change, r.accMax);
    } else if (ha == DEC) {
        la.acc = max(la.acc - change, r.decMax);
    } else {
        if(la.acc < 0)
            la.acc = min(0.0, la.acc + change);
        if(la.acc > 0)
            la.acc = max(0.0, la.acc - change);
    }

    // Induce some additional lesser error
    if(error){
        la.acc += la_error(gen);
    }

    return la;
}



// PHYSICS SIM: Given a current high-level action, apply a motor controller and update observed state. Runs once per time step
Obs physicsModel(State state, Robot& r, double t_step){
    Obs obs = state.obs;
    LA la = state.la;

    double vPrev = obs.vel;
    double xPrev = obs.pos;
    
    // Update velocity and displacement accordingly
    obs.vel = vPrev + la.acc * t_step;

    if(obs.vel < EPSILON){ // Round to 0
        obs.vel = 0;
    }

    if(abs(obs.vel - r.vMax) < EPSILON){ // Round to vMax
        obs.vel = r.vMax;
    }

    if(abs(obs.pos - r.target) < EPSILON){ // Round to target
        obs.pos = r.target;
    }

    obs.pos = xPrev + (obs.vel + vPrev)/2 * t_step;

    return obs;
}

void Robot::runASP(asp* ASP = ASP_Hand_prob){
    this->state.ha = ASP(state, *this);
}

void Robot::updateLA(bool error = true, motor* motor_model = motorModel){
    this->state.la = motor_model(state, *this, error);
}

void Robot::updateObs(phys* phys_model = physicsModel, double t_step = T_STEP){
    this->state.obs = phys_model(state, *this, t_step);
}