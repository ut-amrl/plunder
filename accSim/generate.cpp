#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>

#include "../robot.h"

using namespace std;


/*
 * Configuration
 */

static const int useModel = 0;              // use hand-written ASP (0), LDIPS-generated ASP without error (1), LDIPS-generated ASP with error (2), probabilstic ASP (3)
static const int robotTestSet = 3;          // which robot test set to use (1-3)
static const bool velocityError = true;     // apply error to velocity
static const bool actionError = true;       // apply error to state transitions

// Error distribution parameters
static const double vErrMean = 0.0;         // velocity error distribution
static const double vErrStdDev = 0.1;

// Action error probabilities
static const double haProbCorrect = 0.9;    // Probability of selecting the correct high-level action

// File i/o
static const string pathJson = "accSim/out/data.json";
static const string pathCsv = "accSim/out/data.csv";
static const bool genCsv = true;            // generate CSV trace file
static const bool genJson = true;           // generate JSON trace file

// Global variables
static const double T_STEP = .1;            // time step
static const double T_TOT = 15;             // total time per simulated scenario

// ---------------------------------------------------------------------------------------------------------------------
int main() {

    // Initialization
    double _vErrMean = 0.0;
    double _vErrStdDev = 0.0;
    if(velocityError){
        _vErrMean = vErrMean;
        _vErrStdDev = vErrStdDev;
    }
    normal_distribution<double> vErrDistr(_vErrMean, _vErrStdDev);

    double _haProbCorrect = 1.0;
    if(actionError){
        _haProbCorrect = haProbCorrect;
    }

    // Create some arbitrary robots
    vector<Robot> robots;

    if(robotTestSet == 1){
        robots.push_back(Robot(6, -5, 15, 150, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(3, -3, 30, 100, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(5, -2, 12, 200, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(8, -3, 30, 50, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(1.5, -2, 3, 30, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(3, -2, 4, 20, vErrDistr, _haProbCorrect, useModel)); 
        robots.push_back(Robot(0.5, -1, 2, 15, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(1.5, -2, 40, 300, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(2, -2, 100, 300, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(5, -1, 25, 200, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(10, -10, 25, 500, vErrDistr, _haProbCorrect, useModel));
    } else if(robotTestSet == 2){
        robots.push_back(Robot(5, -2, 15, 80, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(3, -3, 4, 80, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(1.5, -4, 50, 80, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(8, -6, 20, 80, vErrDistr, _haProbCorrect, useModel));
        robots.push_back(Robot(4, -5, 100, 80, vErrDistr, _haProbCorrect, useModel));
    } else if(robotTestSet == 3){
        robots.push_back(Robot(6, -5, 15, 100, vErrDistr, _haProbCorrect, useModel));
    }
    
    // Setup output
    ofstream jsonFile;
    ofstream csvFile;
    if(genJson){
        cout << "generating json\n";
        jsonFile << fixed << setprecision(PRECISION);
        jsonFile.open(pathJson);
        jsonFile << "[";
    }
    if(genCsv){
        cout << "generating csv\n";
        csvFile << fixed << setprecision(PRECISION);
        csvFile.open(pathCsv);
        csvFile << "time, x, v, LA" << "\n";
    }

    // Run simulations and generate json/csv files
    bool first = true;
    for(int i=0; i<robots.size(); i++){
        for(double t=0; t<T_TOT/T_STEP; t++){

            // Run simulation
            string prevHAStr = robots[i].ha == 0 ? "ACC" : 
                                                (robots[i].ha == 1 ? "DEC" : "CON");

            robots[i].updatePhysics(T_STEP);
            robots[i].changeHA();

            string curHAStr = robots[i].ha == 0 ? "ACC" : 
                                        (robots[i].ha == 1 ? "DEC" : "CON");

            // Print trace
            if(genCsv) {
                csvFile << t << ", " << robots[i].x << ", " << robots[i].v << ", " << robots[i].a << "\n";
            }

            if(genJson){
                if(first) first = false;
                else jsonFile << ",";
                jsonFile << R"({"x":{"dim":[1,0,0],"type":"NUM","name":"x","value":)";
                jsonFile << robots[i].x;
                jsonFile << R"(},"v":{"dim":[1,-1,0],"type":"NUM","name":"v","value":)";
                jsonFile << robots[i].v;
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
    }

    if(genJson){
        jsonFile << "]";
        jsonFile.close();
    }
    if(genCsv){
        csvFile.close();
    }

    return 0;
}