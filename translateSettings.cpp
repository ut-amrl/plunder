#include "settings.h"
#include "robot.h"
#include "robotSets.h"

using namespace std;
using namespace SETTINGS;

const uint numSpaces = 30;

// ----- Asserts -------------------------------

// TODO: deprecated
void assertConstraints() {
    if(synthesizer == LDIPS && synth_setting == INCREMENTAL) {
        cout << "LDIPS with incremental synthesis is not supported :(";
        exit(1);
    }
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

    string auxPath = "../includes.h";

    ifstream auxFile;
    auxFile.open(auxPath);

    while(getline(auxFile, res)){
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