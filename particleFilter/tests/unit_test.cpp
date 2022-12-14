// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <math.h>

// #include "../pf_runner.h"
// #include "settings.h"
// #include "accSim/robotSets.h"
// #include "accSim/asps.h"

// using namespace std;

// #define PF_SEED time(0)

// // // ---- OUTDATED -----------------------------------------------------------------------------------------------------------------

// // // Unit tests

// void test_logsumexp(){
//     vector<FLOAT> vec;
//     vec.push_back(-0.69);
//     vec.push_back(-1.6);
//     vec.push_back(-3);

//     FLOAT res = logsumexp(vec);

//     FLOAT sum = 0.0;
//     for(FLOAT each: vec){
//         sum += exp(each);
//     }

//     assert(abs(res - log(sum)) < epsilon);

//     vec.push_back(-1.1);

//     res = logsumexp(vec);
//     sum += exp(vec[vec.size()-1]);

//     assert(abs(res - log(sum)) < epsilon);
// }

// void testSystematicResample(){
//     vector<HA> ha = {ACC, DEC, CON, ACC};
//     vector<FLOAT> weights = {.1, .7, .15, .05};
//     vector<int> ancestor(4);
//     vector<HA> haResampled = systematicResample<HA>(ha, weights, ancestor);

//     cout << "Systematic Resample test: ";
//     for(HA action : haResampled){
//         cout << action << " ";
//     }
//     cout << endl;
// }

// void testLogPdf(){
//     double mu = 1.0;
//     double stddev = 0.5;

//     for(int i = 0; i < 2; i++){
//         double x = ((double) x) / RAND_MAX * 2;
//         double pdf_gaussian = ( 1 / ( stddev * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (x-mu)/stddev, 2.0 ) );

//         assert(abs(log(pdf_gaussian) - logpdf(x, mu, stddev)) < epsilon);
//     }
// }

// void testLikelihood(){
//     Robot r (6, -5, 15, 150, normal_distribution<double>(0.0, 2.0), 0.9);
//     assert(abs(logLikelihoodGivenMotorModel(r, LA { .acc = 5.5 }, ACC, Obs { .pos = 3, .vel = 3 }) + 1.6433567) < 0.01);
//     assert(abs(logLikelihoodGivenMotorModel(r, LA { .acc = 0 }, DEC, Obs { .pos = 3, .vel = 3 }) + 4.7375594) < 0.01);
//     assert(abs(logLikelihoodGivenMotorModel(r, LA { .acc = 0 }, ACC, Obs { .pos = 3, .vel = 3 }) + 6.110248) < 0.01);
// }

// void testTrajectoryRetrieval(){

//     vector<Robot> robots = getRobotSet(robotTestSet, normal_distribution<double>(meanError, stddevError), haProbCorrect);

//     int i = 0;
//     string in = stateGenPath + to_string(i) + ".csv";

//     vector<Obs> dataObs;
//     vector<LA> dataLa;
    
//     if(dataObs.size() == 0 || dataLa.size() == 0){
//         dataObs.clear(); dataLa.clear();
//         readData(in, dataObs, dataLa);
//     }

//     int T = 4;
//     int N = 5;

//     MarkovSystem<HA, LA, Obs, Robot> ms (&sampleInitialHA, ASP_model(model), &logLikelihoodGivenMotorModel, robots[i]);
//     ParticleFilter<HA, LA, Obs, Robot> pf (&ms, dataObs, dataLa);

//     pf.particles = {
//         { ACC, CON, ACC, DEC, CON },
//         { ACC, ACC, DEC, CON, CON },
//         { ACC, CON, DEC, DEC, DEC },
//         { DEC, DEC, CON, ACC, ACC }
//     };
//     pf.ancestors = {
//         { -1, -1, -1, -1, -1 },
//         { 0, 0, 1, 3, 4 },
//         { 0, 1, 2, 3, 4 },
//         { 0, 0, 0, 2, 3 }
//     };

//     vector<vector<HA>> traj = pf.retrieveTrajectories(N);

//     cout << "Trajectory test - trajectories: " << endl;
//     for(vector<HA> each: traj){
//         for(HA ha: each){
//             cout << ha << " ";
//         }
//         cout << endl;
//     }
//     cout << endl;

//     traj.clear();
//     traj = pf.retrieveTrajectories(3);

//     cout << "Trajectory test - trajectories: " << endl;
//     for(vector<HA> each: traj){
//         for(HA ha: each){
//             cout << ha << " ";
//         }
//         cout << endl;
//     }
//     cout << endl;

//     traj.clear();
//     traj = pf.retrieveTrajectories(2);

//     cout << "Trajectory test - trajectories: " << endl;
//     for(vector<HA> each: traj){
//         for(HA ha: each){
//             cout << ha << " ";
//         }
//         cout << endl;
//     }
//     cout << endl;
// }

// void testPF(){
//     int N = 5;
//     int M = 5;
//     double resample_threshold = 0.5;

//     int i = 0;

//     vector<Robot> robots = getRobotSet(robotTestSet, normal_distribution<double>(meanError, stddevError), haProbCorrect);
//     string in = "../../" + stateGenPath + to_string(i) + ".csv";

//     vector<Obs> dataObs;
//     vector<LA> dataLa;
    
//     if(dataObs.size() == 0 || dataLa.size() == 0){
//         dataObs.clear(); dataLa.clear();
//         readData(in, dataObs, dataLa);
//     }

//     vector<vector<HA>> traj = runFilter(N, M, resampleThreshold, robots[i], dataObs, dataLa, ASP_model(model));

//     cout << "PF test - trajectories: " << endl;
//     for(vector<HA> each: traj){
//         for(HA ha: each){
//             cout << ha << " ";
//         }
//         cout << endl;
//     }
//     cout << endl;
// }


// int main(){
//     srand(PF_SEED);

//     test_logsumexp();
//     testSystematicResample();
//     testLogPdf();
//     testLikelihood();

//     testTrajectoryRetrieval();
//     testPF();
// }