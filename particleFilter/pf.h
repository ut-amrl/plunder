#pragma once

#include "robot.h"

using namespace std;
using namespace SETTINGS;

static uint resampCount = 0; // Debug

typedef HA init();
typedef double obs_likelihood(State, Robot&, Obs);

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

// ----- Particle Filter ---------------------------------------------

class ParticleFilter {
public:

    Trajectory state_traj;    // Observed state sequence
    vector<vector<HA>> particles;   // Gives the high-level trajectories of each particle
    vector<vector<int>> ancestors;  // Stores ancestors during resampling

    asp* asp_pf;
    init* init_pf;
    obs_likelihood* obs_likelihood_pf;

    ParticleFilter(Trajectory& _state_traj, asp* _asp, init* _init, obs_likelihood* _obs_likelihood) : state_traj(_state_traj), asp_pf(_asp), init_pf(_init), obs_likelihood_pf(_obs_likelihood) {}

    // Run particle filter on num_particles particles
    double forwardFilter(int num_particles){

        // Initialization
        int N = num_particles;
        int T = state_traj.size();

        vector<double> log_weights(N);
        vector<double> weights(N);
        double log_obs = 0.0;

        for(int t = 0; t < T; t++){
            particles.push_back(vector<HA>(N));
            ancestors.push_back(vector<int>(N, 0));
        }
        
        // Sample from initial distribution
        for(int i = 0; i < N; i++){
            particles[0][i] = init_pf();
            ancestors[0][i] = i;
            log_weights[i] = -log(N);
            weights[i] = exp(log_weights[i]);
        }

        for(int t = 0; t < T; t++){
            
            // Reweight particles
            for(int i = 0; i < N; i++){
                HA x_i = particles[t][i];
                Obs obs = state_traj.get(t).obs;
                obs.acc = (t == 0) ? 0 : state_traj.get(t-1).obs.acc;
                double log_LA_ti = obs_likelihood_pf(State { x_i, obs }, state_traj.r, state_traj.get(t).obs);
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
                particles[t] = systematicResample(particles[t], weights, ancestors[t]);

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
                    particles[t+1][i] = asp_pf(State { particles[t][i], state_traj.get(t).obs }, state_traj.r);
                }
            } else { 
                // resample at last step to eliminate deviating particles
                particles[t] = systematicResample(particles[t], weights, ancestors[t]);
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