#pragma once

#include "domain.h"

using namespace std;

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
HA pointError(HA ha, bool useSafePointError){
    if(usePointError){
        HA prevHA = ha;
        if(!flip(pointAccuracy)){
            int mod = rand() % numHA;
            ha = static_cast<HA>(mod);

            if(useSafePointError){
                if(prevHA == CON && ha == ACC) ha = DEC;
                if(prevHA == DEC) ha = DEC;
            }
        }
    }
    return ha;
}
