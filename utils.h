#pragma once

#include "domain.h"

using namespace std;
using namespace SETTINGS;


// ----- Markov System Definitions --------------------------------
typedef int HA;

struct State {
    HA ha;
    LA la;
    Obs obs;
};

class Robot;

struct Trajectory {
    int T;
    Robot& r;
    vector<State> traj;

    Trajectory(Robot& robot) : T(0), r(robot) {}

    void append(State s) {
        traj.push_back(s);
        T++;
    }

    State get(int t) {
        return traj[t];
    }

    void set(int t, State s){
        traj[t] = s;
    }

    // void append(HA ha){
    //     traj.push_back(State { ha });
    //     T++;
    // }

    // void append(LA la){
    //     traj.push_back(State { HA{}, la });
    //     T++;
    // }

    // void append(Obs obs){
    //     traj.push_back(State { HA{}, LA{}, obs});
    //     T++;
    // }
};

// Helper functions
const uint numHA = HA_Labels.size();

string print(HA ha){
    return HA_Labels[ha];
}

HA to_label(string str){
    for(int i = 0; i < numHA; i++){
        if(HA_Labels[i] == str){
            return i;
        }
    }
    return 0;
}

// Random error distributions
random_device rd;
default_random_engine gen(0);

// Return the distance this robot would travel before stopping if it began decelerating immediately
double DistTraveled(double v, double dec){
    return - v * v / (2 * dec);
}

// Random generator for Bernoulli trial
bool flip(double p){
    double rv = ((double) rand())/RAND_MAX;
    return rv <= p;
}

// evaluate a logistic function
double logistic(double midpoint, double spread, double input){
    return 1.0 / (1.0 + exp(-spread * (input - midpoint)));
}

// randomly transition to another HA
HA pointError(HA ha = to_label(0), double accuracy = POINT_ACCURACY, bool use_safe_transitions = false){
    if(USE_POINT_ERROR){
        HA prevHA = ha;
        if(!flip(accuracy)){
            ha = rand() % numHA;

            // TODO: abstract away safe transitions
        }
    }
    return ha;
}

// Exponentiate some values, take their sum, then take the log of the sum
double logsumexp(vector<double>& vals) {
    if (vals.size() == 0){
        return 0;
    }

    double max_elem = *max_element(vals.begin(), vals.end());
    double sum = accumulate(vals.begin(), vals.end(), 0.0, 
        [max_elem](double a, double b) { return a + exp(b - max_elem); });
    
    return max_elem + log(sum);
}

// Calculate pdf of N(mu, sigma) at x, then take the natural log
double logpdf(double x, double mu, double sigma){
    return (-log(sigma)) - (0.5*log(2*M_PI)) - 0.5*pow((x - mu)/sigma, 2);
}