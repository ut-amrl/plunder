#pragma once

#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>
#include <stdlib.h>

#include "robotSets.h"
#include "asps.h"

using namespace std;

// ----- Configuration: Default Parameters ---------------------------------------------

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
static const string outputPath = "accSim/out/data";   // Output file directory and prefix
                                                // Both the JSON and CSV files will be generated here

// ----- Simulation ---------------------------------------------

void runSim(int robotTestSet, int useModel, double accErrMean, double accErrStdDev, double haProbCorrect){
    
    // Initialization
    normal_distribution<double> accErrDistr(accErrMean, accErrStdDev);
    setModel(useModel);
    vector<Robot> robots = getRobotSet(robotTestSet, accErrDistr, haProbCorrect);
    
    // Setup JSON
    ofstream jsonFile;
    if(genJson){
        cout << "Filling JSON with simulation data:\n";
        jsonFile << fixed << setprecision(PRECISION);
        jsonFile.open(outputPath + ".json");
        jsonFile << "[";
    }

    // Run simulations and generate json/csv files
    bool first = true;
    for(uint i = 0; i < robots.size(); i++){
        
        // Setup CSV file
        ofstream csvFile;
        if(genCsv){
            csvFile << fixed << setprecision(PRECISION);
            csvFile.open(outputPath + to_string(i) + ".csv");
            csvFile << "time, x, v, LA, HA" << "\n";
        }

        // Run simulation
        for(double t = 0; t < T_TOT/T_STEP; t++){

            string prevHAStr = HAToString(robots[i].ha);

            robots[i].updatePhysics(T_STEP);
            robots[i].runASP(ASP_model);

            string curHAStr = HAToString(robots[i].ha);

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
                jsonFile << R"(},"start":{"dim":[0,0,0],"type":"STATE","name":"start","value":")";
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