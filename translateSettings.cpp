#include "settings.h"
#include "robot.h"
#include "robotSets.h"

using namespace std;
using namespace SETTINGS;

const uint numSpaces = 30;

// ----- Asserts -------------------------------

void assertConstraints() {
    // TODO: asserts don't even work
    // assert(STDDEV_ERROR >= 0 && "Standard deviation must be positive");
    // assert(TEMPERATURE >= 0 && "Observation likelihood strength must be greater than 0");

    // assert(POINT_ACCURACY >= 0 && POINT_ACCURACY <= 1 && "HA probability must be between 0 and 1");
    // assert(GEN_ACCURACY >= 0 && GEN_ACCURACY <= 1 && "HA probability must be between 0 and 1");

    // assert(SAMPLE_SIZE <= NUM_PARTICLES && "Sample size must be less than the number of particles");
    // assert(PARTICLES_PLOTTED <= NUM_PARTICLES && "Particles plotted must be less than the number of particles");
    // assert(PLOT_TIME <= T_TOT / T_STEP);
}

// ----- Main -------------------------------

// Reads in settings.h and prints it to settings.txt in standardized format
int main(){
    assertConstraints();

    string hPath = "settings.h";
    string txtPath = "settings.txt";
    string pipsPath = OPTIMIZER_PATH + "/optimizer_settings.py";

    ifstream inFile;
    inFile.open(hPath);
    ofstream outFile;
    outFile.open(txtPath);
    ofstream pipsFile;
    pipsFile.open(pipsPath);

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
            
            string spaces ((numSpaces - varName.size()), ' ');

            // Remove quotation marks
            string valStrOrig = valStr;
            if(typeStr == "string") valStr = valStr.substr(1, valStr.size()-2);
            outFile << varName << spaces << valStr << endl;
            valStr = valStrOrig;
            
            if(valStr == "true" || valStr == "false")
                valStr[0] = toupper(valStr[0]);
            pipsFile << varName << " = " << valStr << endl;
        }
    }

    return 0;
}