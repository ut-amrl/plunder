#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <assert.h>

#include "settings.h"

using namespace std;

// ----- Asserts -------------------------------

void assertConstraints() {
    assert(robotTestSet >= 0 && robotTestSet <= 2 && "Robot test set must be between 0 and 2");
    // assert()
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
            // Remove semicolon
            valStr = valStr.substr(0, valStr.end()-valStr.begin()-1);
            // Remove quotation marks
            if(typeStr == "string") valStr = valStr.substr(1, valStr.end()-valStr.begin()-2);
            outFile << varName << " " << valStr << endl;
        }
    }

    return 0;
}