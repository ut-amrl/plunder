#pragma once

#include "domain.h"

using namespace std;
using namespace SETTINGS;

// Helper functions
const uint numHA = HA_Labels.size();

string print(HA ha){
    return HA_Labels[ha];
}

HA to_label(string str){
    for(uint i = 0; i < numHA; i++){
        if(HA_Labels[i] == str){
            return i;
        }
    }
    return 0;
}

// Define valid transitions
vector<HA> all;
vector<HA> get_valid_ha(HA ha, bool use_safe_transitions=USE_SAFE_TRANSITIONS){
    if(use_safe_transitions){
        return valid_transitions[ha];
    }
    if(all.size() != 0)
        return all;

    for(uint i = 0; i < numHA; i++){
        all.push_back(i);
    }
    return all;
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

// If transition is invalid, randomly choose another one
HA correct(HA prev_ha=0, HA ha=0, bool use_safe_transitions=USE_SAFE_TRANSITIONS) {
    if(!use_safe_transitions){
        return ha;
    }
    
    vector<HA> all_possible_ha = get_valid_ha(prev_ha, use_safe_transitions);
    if(count(all_possible_ha.begin(), all_possible_ha.end(), ha) == 0) {
        int index = rand() % all_possible_ha.size();
        ha = all_possible_ha[index];
    }
    return ha;
}

// randomly transition to another HA
HA pointError(HA prev_ha=0, HA ha=0, double accuracy=POINT_ACCURACY, bool use_safe_transitions=USE_SAFE_TRANSITIONS){
    if(USE_POINT_ERROR){
        if(!flip(accuracy)){
            vector<HA> all_possible_ha = get_valid_ha(prev_ha, use_safe_transitions);
            int index = rand() % all_possible_ha.size();
            ha = all_possible_ha[index];
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

// Trimming a string
inline void ltrim(string &s) {
    s.erase(s.begin(), find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !isspace(ch);
    }));
}
inline void rtrim(string &s) {
    s.erase(find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !isspace(ch);
    }).base(), s.end());
}
inline void trim(string &s) {
    rtrim(s);
    ltrim(s);
}