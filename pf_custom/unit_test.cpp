#include <iostream>
#include <fstream>
#include <sstream>

#include "pf.cpp"
#include "../robot.h"

using namespace std;



// ---------------------------------------------------------------------------------------------------------------------

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





// ---------------------------------------------------------------------------------------------------------------------
// Read low-level action sequence and observed state sequence from file
void readData(vector<Obs>& dataObs, vector<LA>& dataLA){

    ifstream infile;
    infile.open("../accSim/data.csv");
    string res;
    // header line
    getline(infile, res);
    // data lines
    while(getline(infile, res)){

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
    cout << res << endl;

    FLOAT sum = 0.0;
    for(FLOAT each: vec){
        sum += exp(each);
    }

    assert(abs(res - log(sum)) < epsilon);
}

void testSystematicResample(){
    vector<HA> ha = {ACC, DEC, CON, ACC};
    vector<FLOAT> weights = {.1, .7, .15, .05};
    vector<int> ancestor;
    vector<HA> haResampled = systematicResample<HA>(ha, weights, ancestor);
    for(HA action : haResampled){
        cout << action << " ";
    }
    cout << endl;
}





// ---------------------------------------------------------------------------------------------------------------------
void testPF(){
    MarkovSystem<HA, LA, Obs> ms (&sampleInitialHA, &ASP, &logLikelihoodGivenMotorModel);
    vector<Obs> dataObs;
    vector<LA> dataLa;
    readData(dataObs, dataLa);
    PF<HA, LA, Obs> pf (&ms, dataObs, dataLa);
}


int main(){
    srand(time(0));

    test_logsumexp();
    testSystematicResample();
    testPF();
}