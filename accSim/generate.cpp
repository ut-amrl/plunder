#include "generate.h"
#include "settings.h"

using namespace std;

// ----- Main ---------------------------------------------

int main(int argc, char** argv) {
    runSim(robotTestSet, model, genAccuracy, stateGenPath);
    
    return 0;
}