#pragma once

#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>
#include <stdlib.h>

#include "robotSets.h"
#include "robot.h"
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
public:
    Robot &r;
    json x_j;
    json v_j;
    json decMax_j;
    json vMax_j;
    json target_j;
    json start_j;
    json output_j;
    HA prevHA = ACC;

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
        return to_string(t) + ", " + to_string(r.state.obs.pos) + ", " + to_string(r.state.obs.vel)
                + ", " + to_string(r.state.la.acc) + ", " + HAToString(r.state.ha);
    }
    string getJsonRow(){
        x_j["value"] = r.state.obs.pos;
        v_j["value"] = r.state.obs.vel;
        target_j["value"] = r.target;
        vMax_j["value"] = r.vMax;
        decMax_j["value"] = r.decMax;
        start_j["value"] = HAToString(prevHA);
        output_j["value"] = HAToString(r.state.ha);
        prevHA = r.state.ha;

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

void runSim(int robotTestSet, int useModel, double genAccuracy, string outputPath){
    
    // Initialization

    vector<Robot> robots = getRobotSet(robotTestSet);
    
    // Setup JSON
    ofstream jsonFile;
    if(genJson){
        // cout << "Filling JSON with simulation data: " << outputPath << ".json" << endl;

        jsonFile << fixed << setprecision(precision);
        jsonFile.open(outputPath + ".json");
        jsonFile << "[";
    }

    // Run simulations and generate json/csv files
    bool first = true;
    for(uint i = 0; i < numRobots; i++){

        RobotIO rio (robots[i]);
        
        // Setup CSV file
        ofstream csvFile;
        if(genCsv){
            // cout << "Filling CSV with simulation data: " << outputPath << to_string(i) << ".csv" << endl;
            csvFile << fixed << setprecision(precision);
            csvFile.open(outputPath + to_string(i) + ".csv");
            csvFile << rio.getCsvTitles() << "\n";
        }


        // Run simulation
        for(double t = 0; t < T_TOT; t += T_STEP){

            robots[i].updateObs();
            robots[i].runASP(ASP_model(useModel));
            robots[i].state.ha = pointError(robots[i].state.ha, genAccuracy);
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
        }

        if(genCsv){
            csvFile.close();
        }
    }

    if(genJson){
        jsonFile << "]";
        jsonFile.close();
    }

    cout << "--------------Simulation---------------" << endl;
    cout << "Running 1-D kinematic car simulation:\n";
    cout << "Using " << numRobots << " robots and low-level action standard deviation=" << stddevError << endl;
    cout << "Output stored in " << outputPath << "\n\n\n";
}

void executeASP(Robot& r, string outputFile, vector<Obs>& dataObs, asp* asp){

    ofstream outFile;
    outFile.open(outputFile);

    bool pointError = usePointError;
    usePointError = false;

    for(uint n=0; n<particlesPlotted; n++){
        r.reset();
        outFile << r.state.ha << ",";
        for(uint t=1; t<dataObs.size(); t++){
            r.state.obs = dataObs[t];
            r.runASP(asp);
            r.updateLA(false);
            outFile << r.state.ha;
            if(t!=dataObs.size()-1) outFile << ",";
        }
        outFile << endl;
    }
    outFile.close();

    usePointError = pointError;
}
