#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <numeric>

#include "robot.h"
#include "accSim/asps.h"

using namespace std;

#define FLOAT double // Set to double (for precision) or float (for speed)

#define PF_SEED time(0)

static uint resampCount = 0; // Debug

// ----- Helper Functions ---------------------------------------------

// Exponentiate some values, take their sum, then take the log of the sum
FLOAT logsumexp(vector<FLOAT>& vals) {
    if (vals.size() == 0){
        return 0;
    }

    FLOAT max_elem = *max_element(vals.begin(), vals.end());
    FLOAT sum = accumulate(vals.begin(), vals.end(), 0.0, 
        [max_elem](FLOAT a, FLOAT b) { return a + exp(b - max_elem); });
    
    return max_elem + log(sum);
}

// Metric to calculate "effective" particles
FLOAT effectiveParticles(vector<FLOAT>& weights){
    FLOAT sum = 0;
    for(FLOAT x: weights){
        sum += x * x;
    }
    return 1 / sum;
}

// Resample particles given their current weights. 
// This method performs the resampling systematically to reduce variance.
template <typename HA>
vector<HA> systematicResample(vector<HA>& ha, vector<FLOAT>& weights, vector<int>& ancestor){
    resampCount++;
    int n = weights.size();
    vector<FLOAT> cumulativeWeights;
    vector<HA> haResampled;
    FLOAT runningSum = 0;
    for(FLOAT w: weights){
        runningSum += w;
        cumulativeWeights.push_back(runningSum);
    }

    FLOAT interval = 1.0 / (FLOAT) n;
    FLOAT pos = ((FLOAT) rand()) / RAND_MAX * interval; // Initial offset
    for(int i = 0, j = 0; i < n; i++){

        while(cumulativeWeights[j] < pos){
            j++;
        }
        haResampled.push_back(ha[j]);
        ancestor[i] = j;
        pos += interval;
    }
    return haResampled;
}



// ----- Markov System ---------------------------------------------

template<typename HA, typename LA, typename Obs, typename RobotClass>
class MarkovSystem {
    
    private:
    public:

    HA (*sampleInitialHA)(); // Initial distribution
    HA (*ASP)(HA prevHa, Obs prevObs, RobotClass& r); // Provided action-selection policy
    FLOAT (*logLikelihoodGivenMotorModel)(RobotClass& r, LA la, HA ha, Obs obs); // Calculate likelihood of observed LA given the simulated HA sequence
    RobotClass r;

    // Constructor
    MarkovSystem( HA (*_sampleInitialHA)(), 
                  HA (*_ASP)(HA prevHa, Obs prevObs, RobotClass& r),
                  FLOAT (*_logLikelihoodGivenMotorModel)(RobotClass& r, LA la, HA ha, Obs obs),
                  RobotClass& _r):
                        sampleInitialHA(_sampleInitialHA), ASP(_ASP), logLikelihoodGivenMotorModel(_logLikelihoodGivenMotorModel), r(_r)
                  {
    }
    
    // Robot-less constructor
    MarkovSystem( HA (*_sampleInitialHA)(), 
                  HA (*_ASP)(HA prevHa, Obs prevObs, RobotClass& r),
                  FLOAT (*_logLikelihoodGivenMotorModel)(RobotClass& r, LA la, HA ha, Obs obs)):
                        sampleInitialHA(_sampleInitialHA), ASP(_ASP), logLikelihoodGivenMotorModel(_logLikelihoodGivenMotorModel), r()
                  {
    }
};



// ----- Particle Filter ---------------------------------------------

template<typename HA, typename LA, typename Obs, typename RobotClass>
class ParticleFilter {
    
    private:
    public:

    MarkovSystem<HA, LA, Obs, RobotClass>* system;
    vector<Obs> dataObs;    // Observed state sequence
    vector<LA> dataLA;      // Observed low-level action sequence

    vector<vector<HA>> particles;   // Gives the high-level trajectories of each particle
    vector<vector<int>> ancestors;  // Stores ancestors during resampling

    ParticleFilter(MarkovSystem<HA, LA, Obs, RobotClass>* _system, vector<Obs>& _dataObs, vector<LA>& _dataLA){
        system = _system;
        dataObs = _dataObs;
        dataLA = _dataLA;

        assert(dataObs.size() == dataLA.size());
    }

    // Run particle filter on numParticles particles (and some resampleThreshold between 0 and 1)
    void forwardFilter(int numParticles, float resampleThreshold){

        // Initialization
        int N = numParticles;
        int T = dataObs.size();

        vector<FLOAT> log_weights(N);
        vector<FLOAT> weights(N);
        FLOAT log_obs = 0.0;

        for(int t = 0; t < T; t++){
            particles.push_back(vector<HA>(N));
            ancestors.push_back(vector<int>(N, 0));
        }
        
        // Sample from initial distribution
        for(int i = 0; i < N; i++){
            particles[0][i] = system->sampleInitialHA();
            ancestors[0][i] = i;
            log_weights[i] = -log(N);
            weights[i] = exp(log_weights[i]);
        }

        // Extension: iterate through enum calculating possible future HA as optimization. Ex:
        // motionModel(CON, dataObs[t-1]);
        // motionModel(DEC, dataObs[t-1]);
        // motionModel(ACC, dataObs[t-1]);

        for(int t = 0; t < T; t++){
            
            // Reweight particles
            for(int i = 0; i < N; i++){
                HA x_i = particles[t][i];
                FLOAT log_LA_ti = system->logLikelihoodGivenMotorModel(system->r, dataLA[t], x_i, dataObs[t]);
                log_weights[i] += log_LA_ti;
            }

            // DEBUG
            // cout << "Processing time " << t << " and LA " << dataLA[t].acc << endl;
            // for(HA each: particles[t]){
            //     cout << each << " ";
            // }
            // cout << endl;
            // cout << "New weights: ";
            // for(FLOAT each: log_weights){
            //     cout << each << " ";
            // }
            // cout << endl;
            // cout << "New weights: ";
            // for(FLOAT each: log_weights){
            //     cout << exp(each) << " ";
            // }
            // cout << endl;

            // Normalize weights
            FLOAT log_z_t = logsumexp(log_weights);
            FLOAT sum = 0.0;
            for(int i = 0; i < N; i++){
                log_weights[i] -= log_z_t;
                weights[i] = exp(log_weights[i]);
                sum += weights[i];
            }
            assert(abs(sum - 1.0) < robotEpsilon*N);

            // Update log observation likelihood
            log_obs += log_z_t;

            // DEBUG
            // cout << "Normalized weights: ";
            // for(FLOAT each: log_weights){
            //     cout << exp(each) << " ";
            // }
            // cout << endl;

            // Optionally resample when number of effective particles is low
            if(effectiveParticles(weights) < N * resampleThreshold){
                particles[t] = systematicResample<HA>(particles[t], weights, ancestors[t]);

                // Reset weights
                for(int i = 0; i < N; i++){
                    log_weights[i] = -log(N);
                    weights[i] = exp(log_weights[i]);
                }

                // DEBUG
                // cout << "Resampling..." << endl;
                // cout << "New particles: ";
                // for(HA each: particles[t]){
                //     cout << each << " ";
                // }
                // cout << endl;
            } else {
                // Ancestor for each particle is itself
                for(int i = 0; i < N; i++){
                    ancestors[t][i] = i;
                }
            }

            // Forward-propagate particles using provided action-selection policy
            if(t < T-1){
                for(int i = 0; i < N; i++){
                    particles[t+1][i] = system->ASP(particles[t][i], dataObs[t], system->r);
                }
            } else {
                // resample at last step to eliminate deviating particles
                // particles[t] = systematicResample<HA>(particles[t], weights, ancestors[t]);
            }
        }

        cout << "Cumulative observation likelihood: e^" << log_obs << " = " << exp(log_obs) << endl;
    }

    // Retrieve high-level action sequences after running particle filter
    vector<vector<HA>> retrieveTrajectories(int numTrajectories){        
        if(particles.size() == 0){
            cout << "Run the particle filter first!" << endl;
            return vector<vector<HA>>();
        }
        assert(particles.size() == ancestors.size());

        int T = particles.size(); int N = particles[0].size();
        numTrajectories = min(numTrajectories, (int) particles[0].size());

        vector<vector<HA>> trajectories;
        for(int i = 0; i < numTrajectories; i++){
            trajectories.push_back(vector<HA>(T));
        }

        vector<uint> activeParticles;
        for(uint i = 0; (int) activeParticles.size() < numTrajectories; i += (N / numTrajectories)){
            activeParticles.push_back(i);
        }

        for(int t = T - 1; t >= 0; t--){
            for(int i = 0; i < numTrajectories; i++){
                trajectories[i][t] = particles[t][activeParticles[i]];
            }
            if(t != 0){
                for(int i = 0; i < numTrajectories; i++){
                    activeParticles[i] = ancestors[t][activeParticles[i]];
                }
            }
        }

        return trajectories;
    }
};