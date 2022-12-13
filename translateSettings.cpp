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

uint numSpaces = 30;

// ----- Asserts -------------------------------

void assertConstraints() {
    assert(robotTestSet >= 0 && robotTestSet <= 2 && "Robot test set must be between 0 and 2");
    vector<Robot> robots = getRobotSet(robotTestSet, normal_distribution<double>(0, 0), 0);
    assert(numRobots <= robots.size() && "numRobots does not match size of test set");
    assert(model >= 0 && model < ASPs.size() && ("Model must be between 0 and " + (ASPs.size() - 1)));

    assert(stddevError >= 0 && "Standard deviation must be positive");
    assert(obsLikelihoodStrength >= 0 && obsLikelihoodStrength <= 1 && "Observation likelihood strength must be between 0 and 1");

    assert(pointAccuracy >= 0 && pointAccuracy <= 1 && "HA probability must be between 0 and 1");
    assert(genAccuracy >= 0 && genAccuracy <= 1 && "HA probability must be between 0 and 1");
    assert(max_error >= 0 && "max_error must be >= 0");
    // assert(resampleThreshold >= 0 && resampleThreshold <= 1 && "Resample threshold must be between 0 and 1");

    assert(sampleSize <= numParticles && "Sample size must be less than the number of particles");
    assert(particlesPlotted <= numParticles && "Particles plotted must be less than the number of particles");
    assert(timeStepsPlot <= T_TOT / T_STEP);
}

// ----- Main -------------------------------

// Reads in settings.h and prints it to settings.txt in standardized format
int main(int argc, char** argv){
    assertConstraints();

    string basePath = "settings";
    if(argc >= 2) {
        basePath = argv[1];
    }

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
            valStr = valStr.substr(0, valStr.size()-1);
            // Remove quotation marks
            if(typeStr == "string") valStr = valStr.substr(1, valStr.size()-2);
            string spaces ((numSpaces - varName.size()), ' ');
            outFile << varName << spaces << valStr << endl;
        }
    }

    return 0;
}