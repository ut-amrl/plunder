#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>
#include <stdlib.h>

#include "generate.h"
#include "../settings.h"

using namespace std;

// ----- Main ---------------------------------------------

int main(int argc, char** argv) {
    runSim(robotTestSet, model, meanError, stddevError, gen_haProbCorrect, stateGenPath);
    
    return 0;
}