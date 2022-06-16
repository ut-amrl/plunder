#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>
#include <stdlib.h>

#include "generate.h"

using namespace std;

// ----- Main ---------------------------------------------

int main(int argc, char** argv) {
    // Reading parameters
    if(argc > 1){
        if(argc < 6){
            cout << "Please run in the following manner: ./gen <robot test set> <model> <mean error> <error standard deviation> <high-level success rate>" << endl;
            exit(0);
        }

        runSim(stoi(argv[1]), stoi(argv[2]), stod(argv[3]), stod(argv[4]), stod(argv[5]));
    }

    return 0;
}