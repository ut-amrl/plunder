#pragma once

#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>
#include <stdlib.h>

#include "../settings.h"
#include "robotSets.h"
#include "asps.h"

using namespace std;

// ----- Configuration ---------------------------------------------

// Global variables
static const bool genCsv = true;            // generate CSV trace file
static const bool genJson = true;           // generate JSON trace file

// ----- Simulation ---------------------------------------------

void runSim(int robotTestSet, int useModel, double accErrMean, double accErrStdDev, double haProbCorrect, string outputPath){
    
    // Initialization
    normal_distribution<double> accErrDistr(accErrMean, accErrStdDev);
    vector<Robot> robots = getRobotSet(robotTestSet, accErrDistr, haProbCorrect);
    
    // Setup JSON
    ofstream jsonFile;
    if(genJson){
        cout << "Filling JSON with simulation data\n";
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
            robots[i].runASP(ASP_model(useModel));

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