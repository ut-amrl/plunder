#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <numeric>

#include "../robot.h"

using namespace std;

#define FLOAT float

// ---------------------------------------------------------------------------------------------------------------------
FLOAT logsumexp(vector<FLOAT>& vals) {
    if (vals.size() == 0){
        return 0;
    }

    FLOAT max_elem = *max_element(vals.begin(), vals.end());
    FLOAT sum = accumulate(vals.begin(), vals.end(), 0.0, 
        [max_elem](FLOAT a, FLOAT b) { return a + exp(b - max_elem); });
    
    return max_elem + log(sum);
}

FLOAT effectiveParticles(vector<FLOAT>& weights){
    FLOAT sum = 0;
    for(FLOAT x: weights){
        sum += x * x;
    }
    return 1 / sum;
}


template <typename HA>
vector<HA> systematicResample(vector<HA>& ha, vector<FLOAT>& weights, vector<int>& ancestor){
    int n = weights.size();
    vector<FLOAT> cumulativeWeights;
    vector<HA> haResampled;
    FLOAT runningSum = 0;
    for(FLOAT w: weights){
        runningSum += w;
        cumulativeWeights.push_back(runningSum);
    }

    assert(abs(runningSum - 1.0) < epsilon);

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





// ---------------------------------------------------------------------------------------------------------------------
template<typename HA, typename LA, typename Obs, typename RobotClass>
class MarkovSystem {
    
    private:
    public:

    HA (*sampleInitialHA)();
    HA (*ASP)(RobotClass r, HA prevHa, Obs prevObs);
    FLOAT (*logLikelihoodGivenMotorModel)(RobotClass r, LA la, HA ha, Obs obs);
    RobotClass r;

    MarkovSystem( HA (*_sampleInitialHA)(), 
                  HA (*_ASP)(RobotClass r, HA prevHa, Obs prevObs),
                  FLOAT (*_logLikelihoodGivenMotorModel)(RobotClass r, LA la, HA ha, Obs obs),
                  RobotClass _r):
                        sampleInitialHA(_sampleInitialHA), ASP(_ASP), logLikelihoodGivenMotorModel(_logLikelihoodGivenMotorModel), r(_r)
                  {
    }
    
    MarkovSystem( HA (*_sampleInitialHA)(), 
                  HA (*_ASP)(RobotClass r, HA prevHa, Obs prevObs),
                  FLOAT (*_logLikelihoodGivenMotorModel)(RobotClass r, LA la, HA ha, Obs obs)):
                        sampleInitialHA(_sampleInitialHA), ASP(_ASP), logLikelihoodGivenMotorModel(_logLikelihoodGivenMotorModel), r()
                  {
    }
};




// ---------------------------------------------------------------------------------------------------------------------
// Particle filter
template<typename HA, typename LA, typename Obs, typename RobotClass>
class PF {
    
    private:
    public:

    MarkovSystem<HA, LA, Obs, RobotClass>* system;
    vector<Obs> dataObs;
    vector<LA> dataLA;

    vector<vector<HA>> particles;
    vector<vector<int>> ancestors;

    PF(MarkovSystem<HA, LA, Obs, RobotClass>* _system, vector<Obs>& _dataObs, vector<LA>& _dataLA){
        system = _system;
        dataObs = _dataObs;
        dataLA = _dataLA;

        assert(dataObs.size() == dataLA.size());
    }

    void forward_filter(int numParticles, float resampleThreshold){
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

            // Normalization
            FLOAT log_z_t = logsumexp(log_weights);
            FLOAT sum = 0.0;
            for(int i = 0; i < N; i++){
                log_weights[i] -= log_z_t;
                weights[i] = exp(log_weights[i]);
                sum += weights[i];
            }
            assert(abs(sum - 1.0) < epsilon);

            // Update log observation likelihood
            log_obs += log_z_t;

            cout << "WEIGHTS: ";
            for(int i = 0; i < weights.size(); i++){
                cout << weights[i] << " ";
            }
            cout << endl;

            cout << "LOG WEIGHTS: ";
            for(int i = 0; i < log_weights.size(); i++){
                cout << log_weights[i] << " ";
            }
            cout << endl;

            // Optionally resample
            if(effectiveParticles(weights) < N * resampleThreshold){
                cout << "resample at time=" << t << endl;
                particles[t] = systematicResample<HA>(particles[t], weights, ancestors[t]);

                cout << "Ancestors: ";
                for(int i = 0; i < ancestors[t].size(); i++){
                    cout << ancestors[t][i] << " ";
                        // CHANGE MADE HERE
                    log_weights[i] = log_weights[ancestors[t][i]];
                    weights[i] = weights[ancestors[t][i]];
                }
                cout << endl;
            } else {
                // Ancestor for each particle is itself
                for(int i = 0; i < N; i++){
                    ancestors[t][i] = i;
                }
            }

            // Forward-propagate particles
            if(t < T-1){
                for(int i = 0; i < N; i++){
                    particles[t+1][i] = system->ASP(system->r, particles[t][i], dataObs[t]);
                }
            }
        }
    }

    vector<vector<HA>> retrieveTrajectories(){
        if(particles.size() == 0){
            cout << "Run the particle filter first!" << endl;
            return vector<vector<HA>>();
        }
        assert(particles.size() == ancestors.size());

        int T = particles.size(); int N = particles[0].size();

        vector<vector<HA>> trajectories;
        for(int i = 0; i < N; i++){
            trajectories.push_back(vector<HA>(T));
        }

        vector<int> activeParticles(N);
        for(int i = 0; i < N; i++){
            activeParticles[i] = i;
        }

        for(int t = T - 1; t >= 0; t--){
            for(int i = 0; i < N; i++){
                trajectories[i][t] = particles[t][activeParticles[i]];
            }
            if(t != 0){
                for(int i = 0; i < N; i++){
                    activeParticles[i] = ancestors[t][activeParticles[i]];
                }
            }
        }

        return trajectories;
    }

};