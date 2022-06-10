#include <iostream>
#include <fstream>
#include <sstream>
#include "pf.cpp"

using namespace std;





// ---------------------------------------------------------------------------------------------------------------------
struct AccSimObs {
    FLOAT pos;
    FLOAT vel;
};

struct AccSimLA {
    FLOAT acc;
};

enum AccSimHA {
    ACC,
    DEC,
    CON
};

AccSimHA sampleInitialHA(){
    return ACC;
}

AccSimHA ASP(AccSimHA prevHa, AccSimObs prevObs){
    return ACC;
}

FLOAT logLikelihoodGivenMotorModel(AccSimLA la, AccSimHA ha, AccSimObs obs){
    // something gaussian on HA acc to get LA acc
    return 1.0;
}





// ---------------------------------------------------------------------------------------------------------------------
void readData(vector<AccSimObs>& dataObs, vector<AccSimLA>& dataLA){

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

        AccSimObs obs;
        obs.pos = x;
        obs.vel = v;
        dataObs.push_back(obs);

        AccSimLA la;
        la.acc = a;
        dataLA.push_back(la);
    }
}






// ---------------------------------------------------------------------------------------------------------------------
void test_logsumexp(){
    vector<FLOAT> vec;
    vec.push_back(-0.69);
    vec.push_back(-1.6);
    vec.push_back(-3);

    FLOAT res = logsumexp(vec);
    cout << res << endl;
}

void testSystematicResample(){
    vector<AccSimHA> ha = {ACC, DEC, CON, ACC};
    vector<FLOAT> weights = {.1, .7, .15, .05};
    vector<int> ancestor;
    vector<AccSimHA> haResampled = systematicResample<AccSimHA>(ha, weights, ancestor);
    for(AccSimHA action : haResampled){
        cout << action << " ";
    }
    cout << endl;
}





// ---------------------------------------------------------------------------------------------------------------------
void testPF(){
    MarkovSystem<AccSimHA, AccSimObs, AccSimLA> ms (&sampleInitialHA, &ASP, &logLikelihoodGivenMotorModel);
    vector<AccSimObs> dataObs;
    vector<AccSimLA> dataLa;
    readData(dataObs, dataLa);
    PF<AccSimHA, AccSimObs, AccSimLA> pf (&ms, dataObs, dataLa);
}


int main(){
    srand(time(0));
    testSystematicResample();
    testPF();
    test_logsumexp();
}