#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>
#include <stdlib.h>

#include "../robot.h"

using namespace std;

/*
 * Configuration: Default parameters
 */

static int robotTestSet = 3;          // which robot test set to use (1-3)
static int useModel = 2;              // use hand-written ASP (0), LDIPS-generated ASP without error (1), LDIPS-generated ASP with error (2), probabilistic ASP (3)

// Error distribution parameters
static double accErrMean = 0.0;         // acceleration error distribution
static double accErrStdDev = 2.0;

// Action error probabilities
static double haProbCorrect = 0.8;    // Probability of selecting the correct high-level action

// Global variables
static const bool genCsv = true;            // generate CSV trace file
static const bool genJson = true;           // generate JSON trace file
static const double T_STEP = .1;            // time step
static const double T_TOT = 15;             // total time per simulated scenario

// File I/O
static const string path = "accSim/out/data";   // Output file directory and prefix
                                                // Both the JSON and CSV files will be generated here

// ---------------------------------------------------------------------------------------------------------------------
void runSim(int robotTestSet, int useModel, double accErrMean, double accErrStdDev, double haProbCorrect){
    
    // Initialization
    normal_distribution<double> accErrDistr(accErrMean, accErrStdDev);
    
    // Create some robots
    vector<Robot> robots;

    if(robotTestSet == 1){
        robots.push_back(Robot(6, -5, 15, 150, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(3, -3, 30, 100, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(5, -2, 12, 200, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(8, -3, 30, 50, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(1.5, -2, 3, 30, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(3, -2, 4, 20, accErrDistr, haProbCorrect, useModel)); 
        robots.push_back(Robot(0.5, -1, 2, 15, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(1.5, -2, 40, 300, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(2, -2, 100, 300, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(5, -1, 25, 200, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(10, -10, 25, 500, accErrDistr, haProbCorrect, useModel));
    } else if(robotTestSet == 2){
        robots.push_back(Robot(5, -2, 15, 80, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(3, -3, 4, 80, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(1.5, -4, 50, 80, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(8, -6, 20, 80, accErrDistr, haProbCorrect, useModel));
        robots.push_back(Robot(4, -5, 100, 80, accErrDistr, haProbCorrect, useModel));
    } else if(robotTestSet == 3){
        robots.push_back(Robot(5, -4, 12, 100, accErrDistr, haProbCorrect, useModel));
    }
    
    // Setup JSON
    ofstream jsonFile;
    if(genJson){
        cout << "Filling JSON with simulation data:\n";
        jsonFile << fixed << setprecision(PRECISION);
        jsonFile.open(path + ".json");
        jsonFile << "[";
    }

    // Run simulations and generate json/csv files
    bool first = true;
    for(uint i = 0; i < robots.size(); i++){
        
        // Setup CSV file
        ofstream csvFile;
        if(genCsv){
            csvFile << fixed << setprecision(PRECISION);
            csvFile.open(path + to_string(i) + ".csv");
            csvFile << "time, x, v, LA, HA" << "\n";
        }

        // Run simulation
        for(double t = 0; t < T_TOT/T_STEP; t++){

            string prevHAStr = robots[i].ha_tostring();

            robots[i].updatePhysics(T_STEP);
            robots[i].changeHA();

            string curHAStr = robots[i].ha_tostring();

            // Print trace
            if(genCsv) {
                csvFile << t << ", " << robots[i].state.pos << ", " << robots[i].state.vel << ", " << robots[i].la.acc << ", " << curHAStr << "\n";
            }

            if(genJson){
                if(first) first = false;
                else jsonFile << ",";
                jsonFile << R"({"x":{"dim":[1,0,0],"type":"NUM","name":"x","value":)";
                jsonFile << robots[i].state.pos;
                jsonFile << R"(},"v":{"dim":[1,-1,0],"type":"NUM","name":"v","value":)";
                jsonFile << robots[i].state.vel;
                jsonFile << R"(},"target":{"dim":[1,0,0],"type":"NUM","name":"target","value":)";
                jsonFile << robots[i].target;
                jsonFile << R"(},"vMax":{"dim":[1,-1,0],"type":"NUM","name":"vMax","value":)";
                jsonFile << robots[i].vMax;
                jsonFile << R"(},"decMax":{"dim":[1,-2,0],"type":"NUM","name":"decMax","value":)";
                jsonFile << robots[i].decMax;
                jsonFile << R"(},"start":{"dim":[0,0,0],"type":"STATE","name":"output","value":")";
                jsonFile << prevHAStr;
                jsonFile << R"("},"output":{"dim":[0,0,0],"type":"STATE","name":"output","value":")";
                jsonFile << curHAStr;
                jsonFile << R"("}})" << endl;
            }

            if(robots[i].finished()){
                break;
            }
        }

        if(genCsv){
            csvFile.close();
        }
    }

    if(genJson){
        jsonFile << "]";
        jsonFile.close();
    }
}

// runSim() with default parameters
void runSim() {
    runSim(robotTestSet, useModel, accErrMean, accErrStdDev, haProbCorrect);
}


// ---------------------------------------------------------------------------------------------------------------------
int main(int argc, char** argv) {
    // Reading parameters
    if(argc > 1){
        if(argc < 6){
            cout << "Please run in the following manner: ./gen <robot test set> <model> <mean error> <error standard deviation> <high-level success rate>" << endl;
            exit(0);
        }

        robotTestSet = stoi(argv[1]);
        useModel = stoi(argv[2]);
        accErrMean = stod(argv[3]);
        accErrStdDev = stod(argv[4]);
        haProbCorrect = stod(argv[5]);
    }

    runSim(robotTestSet, useModel, accErrMean, accErrStdDev, haProbCorrect);

    return 0;
}