
#include <iostream>
#include "pf.cpp"

using namespace std;



void testSystematicResample(){
    vector<HA> ha = {ACC, DEC, CON, ACC};
    vector<FLOAT> weights = {.1, .7, .15, .05};
    vector<HA> haResampled = systematicResample(ha, weights);
    for(HA action : haResampled){
        cout << action << " ";
    }
    cout << endl;
}



void readData(){

}


void testPF(){
    MarkovSystem ms;
    vector<Obs> dataObs;
    vector<LA> dataLa;
    PF pf (ms, dataObs, dataLa);
}


int main(){
    srand(time(0));
    testSystematicResample();
    testPF();
}