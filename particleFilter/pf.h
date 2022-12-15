#pragma once

#include "robot.h"

using namespace std;

static uint resampCount = 0; // Debug

// ----- Helper Functions ---------------------------------------------

// Metric to calculate "effective" particles
double effectiveParticles(vector<double>& weights){
    double sum = 0;
    for(double x: weights){
        sum += x * x;
    }
    return 1 / sum;
}

// Resample particles given their current weights. 
// This method performs the resampling systematically to reduce variance.
template <typename HA>
vector<HA> systematicResample(vector<HA>& ha, vector<double>& weights, vector<int>& ancestor){
    resampCount++;
    int n = weights.size();
    vector<double> cumulativeWeights;
    vector<HA> haResampled;
    double runningSum = 0;
    for(double w: weights){
        runningSum += w;
        cumulativeWeights.push_back(runningSum);
    }

    double interval = 1.0 / (double) n;
    double pos = ((double) rand()) / RAND_MAX * interval; // Initial offset
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
    HA (*ASP)(State prevState, RobotClass& r); // Provided action-selection policy
    double (*logLikelihoodGivenMotorModel)(State state, RobotClass& r, LA prevLA); // Calculate likelihood of observed LA given the simulated HA sequence
    RobotClass r;

    // Constructor
    MarkovSystem(HA (*_sampleInitialHA)(), 
                HA (*_ASP)(State prevState, RobotClass& r),
                double (*_logLikelihoodGivenMotorModel)(State state, RobotClass& r, LA prevLA),
                RobotClass& _r):

                sampleInitialHA(_sampleInitialHA), ASP(_ASP), logLikelihoodGivenMotorModel(_logLikelihoodGivenMotorModel), r(_r)
    {}
    
    // Robot-less constructor
    MarkovSystem(HA (*_sampleInitialHA)(), 
                HA (*_ASP)(State prevState, RobotClass& r),
                double (*_logLikelihoodGivenMotorModel)(State state, RobotClass& r, LA prevLA)) :
                sampleInitialHA(_sampleInitialHA), ASP(_ASP), logLikelihoodGivenMotorModel(_logLikelihoodGivenMotorModel), r()
    {}
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

    // Run particle filter on NUM_PARTICLES particles (and some RESAMPLE_THRESHOLD between 0 and 1)
    double forwardFilter(int NUM_PARTICLES, float RESAMPLE_THRESHOLD){

        // Initialization
        int N = NUM_PARTICLES;
        int T = dataObs.size();

        vector<double> log_weights(N);
        vector<double> weights(N);
        double log_obs = 0.0;

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

        // cout << "Robot:\n\n\n";

        for(int t = 0; t < T; t++){
            
            // Reweight particles
            for(int i = 0; i < N; i++){
                HA x_i = particles[t][i];
                LA prevLA = (t == 0) ? LA{} : dataLA[t-1];
                double log_LA_ti = system->logLikelihoodGivenMotorModel(State { x_i, prevLA, dataObs[t] }, system->r, dataLA[t]);
                log_weights[i] += log_LA_ti;
            }

            // Normalize weights
            double log_z_t = logsumexp(log_weights);
            double sum = 0.0;
            for(int i = 0; i < N; i++){
                log_weights[i] -= log_z_t;
                weights[i] = exp(log_weights[i]);
                sum += weights[i];
            }
            assert(abs(sum - 1.0) < EPSILON*N);

            // Update log observation likelihood
            log_obs += log_z_t;

            // Optionally resample when number of effective particles is low
            if(effectiveParticles(weights) < N * RESAMPLE_THRESHOLD){
                particles[t] = systematicResample<HA>(particles[t], weights, ancestors[t]);

                // Reset weights
                for(int i = 0; i < N; i++){
                    log_weights[i] = -log(N);
                    weights[i] = exp(log_weights[i]);
                }
            } else {
                // Ancestor for each particle is itself
                for(int i = 0; i < N; i++){
                    ancestors[t][i] = i;
                }
            }

            // Forward-propagate particles using provided action-selection policy
            if(t < T-1){
                for(int i = 0; i < N; i++){
                    particles[t+1][i] = system->ASP(State { particles[t][i], LA {}, dataObs[t] } , system->r);
                }
            } else { 
                // resample at last step to eliminate deviating particles
                particles[t] = systematicResample<HA>(particles[t], weights, ancestors[t]);
            }
        }
        return log_obs;
    }

    // Retrieve high-level action sequences after running particle filter
    vector<vector<HA>> retrieveTrajectories(vector<vector<HA>>& trajectories, int num_trajectories){        
        if(particles.size() == 0){
            cout << "Run the particle filter first!" << endl;
            return vector<vector<HA>>();
        }
        assert(particles.size() == ancestors.size());

        int T = particles.size(); int N = particles[0].size();
        num_trajectories = min(num_trajectories, (int) particles[0].size());

        while(trajectories.size() < num_trajectories){
            trajectories.push_back(vector<HA>(T));
        }

        vector<uint> activeParticles;
        for(uint i = 0; (int) activeParticles.size() < num_trajectories; i += (N / num_trajectories)){
            activeParticles.push_back(i);
        }

        for(int t = T - 1; t >= 0; t--){
            for(int i = 0; i < num_trajectories; i++){
                trajectories[i][t] = particles[t][activeParticles[i]];
            }
            if(t != 0){
                for(int i = 0; i < num_trajectories; i++){
                    activeParticles[i] = ancestors[t][activeParticles[i]];
                }
            }
        }

        return trajectories;
    }

    vector<vector<HA>> smoothTrajectories(vector<vector<HA>>& trajectories){  
        for(vector<HA>& traj : trajectories){
            for(int i = 1; i < traj.size() - 1; i++){
                if(traj[i-1] == traj[i+1] && traj[i-1] != traj[i]){ // Apply smoothening
                    traj[i] = traj[i-1];
                }
            }
        }

        return trajectories;
    }
};