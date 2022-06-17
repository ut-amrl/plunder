#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <string>
#include <cstring>

#include "pf_runner.h"

using namespace std;

#define PF_SEED time(0)

// ----- Default Configuration ---------------------------------------------

static vector<Robot> robots = { Robot(5, -4, 12, 100, normal_distribution<double>(0.0, 1.0), 0.8) };
static int N = 1000;
static double resampleThreshold = 0.1;

// File paths
static string inputPath = "accSim/out/data";
static string outputPath = "particleFilter/out/pf";

// ----- Main ---------------------------------------------

int main(int argc, char** argv){
    // Reading parameters
    if(argc > 1){
        if(argc < 8){
            cout << "Please run in the following manner: ./pf <robot test set> <mean error> <error std dev> <high-level success rate> <model> <numParticles> <resampleThreshold>" << endl;
            exit(0);
        }

        vector<Robot> robots = getRobotSet(stoi(argv[1]), normal_distribution<double>(stod(argv[2]), stod(argv[3])), stod(argv[4]));
        setModel(stod(argv[5]));

        processPath(stoi(argv[6]), stod(argv[7]), robots, inputPath, outputPath, robots.size());
    }
}