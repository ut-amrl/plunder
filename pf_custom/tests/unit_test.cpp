#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>

#include "../pf.cpp"

using namespace std;

#define PF_SEED time(0)

// ---------------------------------------------------------------------------------------------------------------------

// Initial distribution
HA sampleInitialHA(){
    // Distribution 1: Point distribution
    // return ACC;

    // Distribution 2: Uniformly Randomly Distributed
    double rv = ((double) rand()) / RAND_MAX;
    if(rv <= 0.33){
        return ACC;
    } else if (rv <= 0.67){
        return DEC;
    } else {
        return CON;
    }
}

// Run robot-based action-selection policy
HA ASP(Robot r, HA prevHA, Obs prevObs){
    r.x = prevObs.pos;
    r.v = prevObs.vel;
    r.ha = prevHA;

    r.changeHA();
    
    return r.ha;
}

// Calculate pdf of N(mu, sigma) at x, then take the natural log
FLOAT logpdf(FLOAT x, FLOAT mu, FLOAT sigma){
    return (-log(sigma)) - (0.5*log(2*M_PI)) - 0.5*pow((x - mu)/sigma, 2);
}

// Calculate probability of observing given LA with a hypothesized high-level action, then take natural log
FLOAT logLikelihoodGivenMotorModel(Robot r, LA la, HA ha, Obs obs){
    double la_mean = (ha == ACC) ? r.accMax : (ha == DEC) ? r.decMax : 0;
    double res = logpdf(la.acc, la_mean, 5.0);
    return res;
}



// ---------------------------------------------------------------------------------------------------------------------

// Read low-level action sequence and observed state sequence from file
void readData(vector<Obs>& dataObs, vector<LA>& dataLA){

    ifstream infile;
    infile.open("../accSim/data.csv");
    string res;
    // header line
    getline(infile, res);
    // data lines
    for(int i = 0; i < 20; i++){
        if(!getline(infile, res))
            break;
        istringstream iss (res);
        float time; string comma1;
        float x; string comma2;
        float v; string comma3;
        float a;
        iss >> time >> comma1 >> x >> comma2 >> v >> comma3 >> a;

        Obs obs;
        obs.pos = x;
        obs.vel = v;
        dataObs.push_back(obs);

        LA la;
        la.acc = a;
        dataLA.push_back(la);
    }
}






// ---------------------------------------------------------------------------------------------------------------------

// Unit tests

void test_logsumexp(){
    vector<FLOAT> vec;
    vec.push_back(-0.69);
    vec.push_back(-1.6);
    vec.push_back(-3);

    FLOAT res = logsumexp(vec);

    FLOAT sum = 0.0;
    for(FLOAT each: vec){
        sum += exp(each);
    }

    assert(abs(res - log(sum)) < epsilon);

    vec.push_back(-1.1);

    res = logsumexp(vec);
    sum += exp(vec[vec.size()-1]);

    assert(abs(res - log(sum)) < epsilon);
}

void testSystematicResample(){
    vector<HA> ha = {ACC, DEC, CON, ACC};
    vector<FLOAT> weights = {.1, .7, .15, .05};
    vector<int> ancestor(4);
    vector<HA> haResampled = systematicResample<HA>(ha, weights, ancestor);

    cout << "Systematic Resample test: ";
    for(HA action : haResampled){
        cout << action << " ";
    }
    cout << endl;
}

void testLogPdf(){
    double mu = 1.0;
    double stddev = 0.5;

    for(int i = 0; i < 2; i++){
        double x = ((double) x) / RAND_MAX * 2;
        double pdf_gaussian = ( 1 / ( stddev * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (x-mu)/stddev, 2.0 ) );

        assert(abs(log(pdf_gaussian) - logpdf(x, mu, stddev)) < epsilon);
    }
}

void testLikelihood(){
    Robot r (6, -5, 15, 150, normal_distribution<double>(0.0, 0.1), 0.9, 0);
    // assert(abs(logLikelihoodGivenMotorModel(r, LA { .acc = 5.5 }, ACC, Obs { .pos = 3, .vel = 3 }) + 2.03143971076) < epsilon);
    // assert(abs(logLikelihoodGivenMotorModel(r, LA { .acc = 0 }, DEC, Obs { .pos = 3, .vel = 3 }) + 3.40643971076) < epsilon);
    // assert(abs(logLikelihoodGivenMotorModel(r, LA { .acc = 0 }, ACC, Obs { .pos = 3, .vel = 3 }) + 4.01755082187) < epsilon);
}

void testTrajectoryRetrieval(){
    MarkovSystem<HA, LA, Obs, Robot> ms (&sampleInitialHA, &ASP, &logLikelihoodGivenMotorModel);
    vector<Obs> dataObs;
    vector<LA> dataLa;
    readData(dataObs, dataLa);
    PF<HA, LA, Obs, Robot> pf (&ms, dataObs, dataLa);
    
    int T = 4;
    int N = 5;

    pf.particles = {
        { ACC, CON, ACC, DEC, CON },
        { ACC, ACC, DEC, CON, CON },
        { ACC, CON, DEC, DEC, DEC },
        { DEC, DEC, CON, ACC, ACC }
    };
    pf.ancestors = {
        { -1, -1, -1, -1, -1 },
        { 0, 0, 1, 3, 4 },
        { 0, 1, 2, 3, 4 },
        { 0, 0, 0, 2, 3 }
    };

    vector<vector<HA>> traj = pf.retrieveTrajectories();

    cout << "Trajectory test - trajectories: " << endl;
    for(vector<HA> each: traj){
        for(HA ha: each){
            cout << ha << " ";
        }
        cout << endl;
    }
    cout << endl;
}

void testPF(){
    int N = 5;
    double resample_threshold = 0.5;

    Robot r (6, -5, 15, 150, normal_distribution<double>(0.0, 0.1), 1, 0);
    MarkovSystem<HA, LA, Obs, Robot> ms (&sampleInitialHA, &ASP, &logLikelihoodGivenMotorModel, r);
    vector<Obs> dataObs;
    vector<LA> dataLa;
    readData(dataObs, dataLa);

    PF<HA, LA, Obs, Robot> pf (&ms, dataObs, dataLa);
    pf.forward_filter(N, resample_threshold);
    
    vector<vector<HA>> traj = pf.retrieveTrajectories();
    cout << "PF test - trajectories: " << endl;
    for(vector<HA> each: traj){
        for(HA ha: each){
            cout << ha << " ";
        }
        cout << endl;
    }
    cout << endl;
}


int main(){
    srand(PF_SEED);

    test_logsumexp();
    testSystematicResample();
    testLogPdf();
    testLikelihood();

    testTrajectoryRetrieval();
    testPF();
}