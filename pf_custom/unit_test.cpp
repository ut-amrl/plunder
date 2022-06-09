#include <iostream>
#include <fstream>
#include <sstream>
#include "pf.cpp"

using namespace std;

void test_logsumexp(){
    vector<FLOAT> vec;
    vec.push_back(-0.69);
    vec.push_back(-1.6);
    vec.push_back(-3);

    FLOAT res = logsumexp(vec);
    cout << res << endl;
}

void testSystematicResample(){
    // vector<HA> ha = {ACC, DEC, CON, ACC};
    // vector<FLOAT> weights = {.1, .7, .15, .05};
    // vector<HA> haResampled = systematicResample(ha, weights);
    // for(HA action : haResampled){
    //     cout << action << " ";
    // }
    // cout << endl;
}



void readData(vector<Obs>& dataObs, vector<LA>& dataLA){

    ifstream infile;
    infile.open("../accSim/data.csv");
    string res;
    // header line
    getline(infile, res);
    // data lines
    while(getline(infile, res)){

        istringstream iss (res);
        float time;
        string comma1;
        float x;
        string comma2;
        float v;
        string comma3;
        float val;
        iss >> time >> comma1 >> x >> comma2 >> v >> comma3 >> val;

        Obs obs;
        obs.pos = x;
        obs.vel = v;
        dataObs.push_back(obs);

        LA la;
        la.acc = val;
        dataLA.push_back(la);
    }
    for(Obs ob : dataObs){
        cout << ob.vel << endl;
        cout << ob.pos << endl;
    }
    for(LA la : dataLA){
        cout << la.acc << endl;
    }

}


void testPF(){
    MarkovSystem ms;
    vector<Obs> dataObs;
    vector<LA> dataLa;
    readData(dataObs, dataLa);
    PF pf (ms, dataObs, dataLa);
}


int main(){
    srand(time(0));
    testSystematicResample();
    testPF();
    test_logsumexp();
}