#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <numeric>

using namespace std;

#define FLOAT float
#define epsilon 10E-6


// ---------------------------------------------------------------------------------------------------------------------
struct Obs {
    FLOAT pos;
    FLOAT vel;
};

struct LA {
    FLOAT acc;
};

enum HA {
    ACC,
    DEC,
    CON
};

FLOAT logsumexp(vector<FLOAT>& vals) {
    if (vals.size() == 0){
        return 0;
    }

    FLOAT max_elem = *max_element(vals.begin(), vals.end());
    FLOAT sum = accumulate(vals.begin(), vals.end(), 0.0, 
        [max_elem](FLOAT a, FLOAT b) { return a + exp(b - max_elem); });
    
    return max_elem + log(sum);
}


// ---------------------------------------------------------------------------------------------------------------------
FLOAT effectiveParticles(vector<FLOAT>& weights){
    FLOAT sum = 0;
    for(FLOAT x: weights){
        sum += x * x;
    }
    return 1 / sum;
}


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
        ancestor.push_back(j);
        pos += interval;
    }
    return haResampled;
}





// ---------------------------------------------------------------------------------------------------------------------
// TODO: make into abstract class
class MarkovSystem {
    
    private:
    public:

    HA sampleInitialHA(){
        return ACC;
    }

    HA ASP(HA prevHa, Obs prevObs){
        return ACC;
    }

    FLOAT logLikelihoodGivenMotorModel(LA la, HA ha, Obs obs){
        // something gaussian on HA acc to get LA acc
        return 1.0;
    }
    
};



// ---------------------------------------------------------------------------------------------------------------------
class PF {
    
    private:
    public:

    MarkovSystem system;
    vector<Obs> dataObs;
    vector<LA> dataLA;

    PF(MarkovSystem _system, vector<Obs> _dataObs, vector<LA> _dataLA){
        system = _system;
        dataObs = _dataObs;
        dataLA = _dataLA;

        assert(dataObs.size() == dataLA.size());
    }

    void forward_filter(int numParticles, float resampleThreshold){
        int N = numParticles;
        int T = dataObs.size();

        vector<vector<HA>> particles;
        vector<vector<int>> ancestors;

        vector<FLOAT> log_weights(N);
        vector<FLOAT> weights(N);
        FLOAT log_obs = 0.0;


        // Sample from initial distribution
        for(int t = 0; t < T; t++){
            particles.push_back(vector<HA>(N));
            ancestors.push_back(vector<int>(N));
        }
        
        for(int i = 0; i < N; i++){
            particles[0][i] = system.sampleInitialHA();
            ancestors[0][i] = -1;
            log_weights[i] = -log(N);
            weights[i] = exp(log_weights[i]);
        }

        // iterate through enum calculating possible future HA as optimization
        // EX: 
        // motionModel(CON, dataObs[t-1]);
        // motionModel(DEC, dataObs[t-1]);
        // motionModel(ACC, dataObs[t-1]);

        for(int t = 1; t < T; t++){
            
            // Reweight particles
            for(int i = 0; i < N; i++){
                HA x_i = particles[t][i];
                FLOAT log_LA_ti = system.logLikelihoodGivenMotorModel(dataLA[t], x_i, dataObs[t]);
                log_weights[i] += log_LA_ti;
            }

            // Normalization
            FLOAT log_z_t = logsumexp(log_weights);
            FLOAT sum = 0.0;
            for(int i = 0; i < N; i++){
                log_weights[i] -= log_z_t;
                sum += log_weights[i];
                weights[i] = exp(log_weights[i]);
            }
            assert(abs(sum - 1.0) < epsilon);

            // Update log observation likelihood
            log_obs += log_z_t;

            // Optionally resample
            if(effectiveParticles(weights) < N * resampleThreshold){
                particles[t] = systematicResample(particles[t], weights, ancestors[t]);
            } else {
                // Ancestor for each particle is itself
                for(int i = 0; i < N; i++){
                    ancestors[t][i] = i;
                }
            }

            // Forward-propagate particles
            if(t < T-1){
                for(int i = 0; i < N; i++){
                    particles[t+1][i] = system.ASP(particles[t][i], dataObs[t]);
                }
            }
            
        }
    }

};