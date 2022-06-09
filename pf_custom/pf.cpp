#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;

#define FLOAT float


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


FLOAT effectiveParticles(vector<FLOAT> weights){
    FLOAT sum = 0;
    for(FLOAT x: weights){
        sum += x * x;
    }
    return 1 / sum;
}


vector<HA> systematicResample(vector<HA> ha, vector<FLOAT> weights){
    int n = weights.size();
    vector<FLOAT> cumulativeWeights;
    vector<HA> haResampled;
    FLOAT runningSum = 0;
    for(FLOAT w: weights){
        runningSum += w;
        cumulativeWeights.push_back(runningSum);
    }
    FLOAT interval = 1.0 / (FLOAT) n;
    FLOAT offset = ((FLOAT) rand()) / RAND_MAX * interval;
    FLOAT pos = offset;
    for(int i=0; i<n; i++){
        int index = lower_bound(cumulativeWeights.begin(), cumulativeWeights.end(), pos) - cumulativeWeights.begin();
        haResampled.push_back(ha.at(index));
        pos += interval;
    }
    return haResampled;
}


// TODO: make into abstract class
class MarkovSystem {
    
    private:
    public:

    HA sampleInitialHA(){
        return ACC;
    }

    // ASP
    HA motionModel(HA prevHa, Obs prevObs){
        return ACC;
    }

    // Motor model
    LA observationModel(HA ha, Obs obs){
        LA la;
        la.acc = 0;
        return la;
    }

    FLOAT obsLikelihood(LA la, HA ha, Obs obs){
        return 0;
    }
    
};



class PF {
    
    private:
    public:

    MarkovSystem system;
    vector<Obs> dataObs;
    vector<LA> dataLa;
    vector<vector<HA>> particles;
    vector<vector<int>> ancestors;


    PF(MarkovSystem _system, vector<Obs> _dataObs, vector<LA> _dataLa){
        system = _system;
        dataObs = _dataObs;
        dataLa = _dataLa;
    }


    void forward_filter(int numParticles, int time_steps, float resampleThreshold, FLOAT (*score_f) (Obs, Obs)){
        int N = numParticles;
        int T = time_steps;

        vector<FLOAT> log_weights(N);
        // Sample from initial distribution
        for(int i = 0; i < N; i++){
            particles.push_back(vector<HA>(N));
            ancestors.push_back(vector<int>(N));
            particles[0][i] = system.sampleInitialHA();
            ancestors[0][i] = -1;
            log_weights[i] = -log(N);
        }
    }

};