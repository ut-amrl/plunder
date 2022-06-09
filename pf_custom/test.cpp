
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


int main(){
    srand(time(0));
    cout << "hello" << endl;
    testSystematicResample();
}