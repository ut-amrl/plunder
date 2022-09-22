#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <assert.h>

#include "settings.h"
#include "robot.h"
#include "accSim/robotSets.h"
#include "accSim/asps.h"

using namespace std;

uint numSpaces = 20;

// ----- Asserts -------------------------------

void assertConstraints() {
    assert(robotTestSet >= 0 && robotTestSet <= 2 && "Robot test set must be between 0 and 2");
    vector<Robot> robots = getRobotSet(robotTestSet, normal_distribution<double>(0, 0), 0);
    assert(numRobots <= robots.size() && "numRobots does not match size of test set");
    assert(model >= 0 && model < ASPs.size() && ("Model must be between 0 and " + (ASPs.size() - 1)));

    assert(stddevError >= 0 && "Standard deviation must be positive");
    assert(pf_stddevError >= 0 && "Standard deviation must be positive");
    assert(pf_stddevError >= stddevError && "PF stddev must be greater than actual stddev");

    assert(pointAccuracy >= 0 && pointAccuracy <= 1 && "HA probability must be between 0 and 1");
    assert(genAccuracy >= 0 && genAccuracy <= 1 && "HA probability must be between 0 and 1");
    assert(max_error >= 0 && "min_accuracy must be between 0 and 1");
    // assert(resampleThreshold >= 0 && resampleThreshold <= 1 && "Resample threshold must be between 0 and 1");

    assert(sampleSize <= numParticles && "Sample size must be less than the number of particles");
    assert(particlesPlotted <= numParticles && "Particles plotted must be less than the number of particles");
}

// ----- Main -------------------------------

// Reads in settings.h and prints it to settings.txt in standardized format
int main(int argc, char** argv){
    assertConstraints();

    if(argc < 2) {
        cout << "please input which settings file to process" << endl;
        exit(1);
    }

    string basePath = argv[1];
    string hPath = basePath + ".h";
    string txtPath = basePath + ".txt";

    ifstream inFile;
    inFile.open(hPath);
    ofstream outFile;
    outFile.open(txtPath);

    string res;
    while(getline(inFile, res)){
        istringstream iss (res);
        string constStr, typeStr, varName, equalsStr, valStr;
        iss >> constStr;
        if(constStr == "const"){
            iss >> typeStr >> varName >> equalsStr >> valStr;
            // shortcut, ignore if there are spaces on the RHS
            if(valStr.back() != ';') continue;
            // Remove semicolon
            valStr = valStr.substr(0, valStr.end()-valStr.begin()-1);
            // Remove quotation marks
            if(typeStr == "string") valStr = valStr.substr(1, valStr.end()-valStr.begin()-2);
            string spaces ((numSpaces - (varName.end()-varName.begin())), ' ');
            outFile << varName << spaces << valStr << endl;
        }
    }

    return 0;
}