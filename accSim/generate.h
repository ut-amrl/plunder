#pragma once

#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>
#include <stdlib.h>

#include "settings.h"
#include "robotSets.h"
#include "asps.h"
#include "ast/ast.hpp"

using nlohmann::json;
using namespace std;
using namespace AST;

// ----- Configuration ---------------------------------------------

// Global variables
static const bool genCsv = true;            // generate CSV trace file
static const bool genJson = true;           // generate JSON trace file

// ----- Simulation ------------------------------------------------

json fillJson(vector<int> dim, string type, string name){
    json j;
    j["dim"] = dim;
    j["type"] = type;
    j["name"] = name;
    return j;
}

class RobotIO {
    private:
    Robot &r;
    json x_j;
    json v_j;
    json decMax_j;
    json vMax_j;
    json target_j;
    json start_j;
    json output_j;
    string prevHAStr = "ACC";

    public:
    RobotIO(Robot &_r): r(_r) {
        x_j = fillJson(vector<int>{1, 0, 0}, "NUM", "x");
        v_j = fillJson(vector<int>{1, -1, 0}, "NUM", "v");
        decMax_j = fillJson(vector<int>{1, -2, 0}, "NUM", "decMax");
        vMax_j = fillJson(vector<int>{1, -1, 0}, "NUM", "vMax");
        target_j = fillJson(vector<int>{1, 0, 0}, "NUM", "target");
        start_j = fillJson(vector<int>{0, 0, 0}, "STATE", "start");
        output_j = fillJson(vector<int>{0, 0, 0}, "STATE", "output");
    }
    string getCsvTitles(){
        return "time, x, v, LA, HA";
    }
    string getCsvRow(double t){
        return to_string(t) + ", " + to_string(r.state.pos) + ", " + to_string(r.state.vel)
                + ", " + to_string(r.la.acc) + ", " + HAToString(r.ha);
    }
    string getJsonRow(){
        x_j["value"] = r.state.pos;
        v_j["value"] = r.state.vel;
        target_j["value"] = r.target;
        vMax_j["value"] = r.vMax;
        decMax_j["value"] = r.decMax;
        start_j["value"] = prevHAStr;
        output_j["value"] = HAToString(r.ha);
        prevHAStr = HAToString(r.ha);

        json all_j;
        all_j["x"] = x_j;
        all_j["v"] = v_j;
        all_j["decMax"] = decMax_j;
        all_j["vMax"] = vMax_j;
        all_j["target"] = target_j;
        all_j["start"] = start_j;
        all_j["output"] = output_j;
        return to_string(all_j);
    }
};

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

        RobotIO rio (robots[i]);
        
        // Setup CSV file
        ofstream csvFile;
        if(genCsv){
            csvFile << fixed << setprecision(PRECISION);
            csvFile.open(outputPath + to_string(i) + ".csv");
            csvFile << rio.getCsvTitles() << "\n";
        }

        // Run simulation
        for(double t = 0; t < T_TOT; t += T_STEP){

            robots[i].updatePhysics(T_STEP);
            robots[i].runASP(ASP_model(useModel));
            robots[i].ha = pointError(ACC, robots[i].ha, robots[i], false);
            robots[i].updateLA();


            // Print trace
            if(genCsv) {
                csvFile << rio.getCsvRow(t) << "\n";
            }

            if(genJson){
                if(first) first = false;
                else jsonFile << ",";
                jsonFile << rio.getJsonRow() << endl;
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