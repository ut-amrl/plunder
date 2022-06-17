#include <dlfcn.h>
#include <z3++.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "ast/ast.hpp"
#include "ast/enumeration.hpp"
#include "ast/library_functions.hpp"
#include "ast/parsing.hpp"
#include "visitors/interp_visitor.hpp"
#include "ast/synthesis.hpp"

#include "../robot.h"
#include "../particleFilter/pf_runner.h"

using namespace std;
using namespace AST;
using namespace z3;
using Eigen::Vector2f;
using json = nlohmann::json;

// ----- Default Configuration ---------------------------------------------

const string obsDataPath = "accSim/out/data0.csv";
const string aspPathBase = "synthesis/";
const string hiLvlDataPath = "particleFilter/out/pf.csv";
const string operationLibPath = "pips/ops/test_library.json";

// EM Loop parameters
const int numIterations = 10;

// LDIPS parameters
const int window_size = 0;
const int feature_depth = 3;
const int sketch_depth = 3;
const float min_accuracy = 0.95;

// Particle filter parameters
const int pfN = 1000;
const float pfResampleThreshold = 0.2;


vector<Obs> dataObs;
vector<LA> dataLA;

unordered_set<Var> variables;
vector<pair<string,string>> transitions;
vector<ast_ptr> preds;
vector<FunctionEntry> library;

Robot globalRobot;
asp_t* curASP_em;

namespace std {
    ostream& operator<<(ostream& os, const AST::ast_ptr& ast);
}

Example dataToExample(HA ha, Obs state, Robot* robot){
    Example ex;
    SymEntry xEntry((float) state.pos);
    SymEntry vEntry((float) state.vel);
    SymEntry curHAEntry(HAToString(ha));
    SymEntry decMaxEntry((float) robot->decMax);
    SymEntry vMaxEntry((float) robot->vMax);
    SymEntry targetEntry((float) robot->target);
    SymEntry zeroVelEntry((float) 0);
    ex.symbol_table_["x"] = xEntry;
    ex.symbol_table_["v"] = vEntry;
    ex.symbol_table_["decMax"] = decMaxEntry;
    ex.symbol_table_["vMax"] = vMaxEntry;
    ex.symbol_table_["target"] = targetEntry;
    ex.symbol_table_["zeroVel"] = zeroVelEntry;

    ex.start_ = curHAEntry;
    return ex;
}

HA transitionUsingASPTree(HA ha, Obs state, Robot* robot){
    Example obsObject = dataToExample(ha, state, robot);
    for(uint i = 0; i < transitions.size(); i++){
        if(HAToString(ha) == transitions[i].first){
            if(InterpretBool(preds[i], obsObject)) {
                ha = stringToHA(transitions[i].second);
                return putErrorIntoHA(ha, robot);
            }
        }
    }
    // we only upload transitions that change state, 
    // so it is assumed that the "else" clause corresponds to staying in the same state
    return putErrorIntoHA(ha, robot);
}

// Initial ASP: random transitions
HA initialASP(HA ha, Obs state, Robot* r){
    // return putErrorIntoHA(ASP_Hand(ha, state, r), r);
    // if(r->sampleDiscrete(0.33)){
    //     ha = ACC;
    // } else if (r->sampleDiscrete(0.5)){
    //     ha = DEC;
    // } else {
    //     ha = CON;
    // }
    // return ha;
    return ASP_random(ha, state, r);
}

vector<Example> expectation(Robot* robot, uint iteration){
    // Run filter
    vector<vector<HA>> trajectories = runFilter(pfN, pfResampleThreshold, robot, dataObs, dataLA, curASP_em);

    cout << "synthesis/out/examples/pf" + to_string(iteration) + ".csv" << endl;
    writeData("synthesis/out/examples/pf" + to_string(iteration) + ".csv", robot, trajectories);

    // Convert each particle trajectory point to LDIPS-supported Example
    vector<Example> examples;
    for(int n = 0; n < pfN; n++){
        vector<HA> traj = trajectories[n];
        for(int t = 0; t < dataObs.size() - 1; t++){
            Example ex = dataToExample(traj[t], dataObs[t], robot);

            // Provide next high-level action
            ex.result_ = SymEntry(HAToString(traj[t+1]));
            examples.push_back(ex);
        }
    }
    return examples;
}

void maximization(vector<Example> examples, uint iteration){


    // this is where we can optimize and remove the last transition of each if-statement
    // we can also order the transitions however we want
    
    examples = WindowExamples(examples, window_size);
    
    // Turn variables into roots
    vector<ast_ptr> inputs, roots;
    for (const Var& variable : variables) {
        roots.push_back(make_shared<Var>(variable));
    }
    

    vector<Signature> signatures;
    vector<ast_ptr> ops = RecEnumerate(roots, inputs, examples, library, feature_depth, &signatures);


    // -----------------
    cout << "----Roots----" << endl;
    for (auto& node : roots) {
        cout << node << endl;
    }
    cout << endl;

    cout << "----Transitions----" << endl;
    for (auto& trans : transitions) {
        cout << trans.first << "->" << trans.second << endl;
    }
    cout << endl;
    cout << "---- Features Synthesized ----" << endl;
    for (auto& feat : ops) {
        cout << feat << endl;
    }
    cout << endl;

    cout << "---- Number of Features Enumerated ----" << endl;
    cout << ops.size() << endl << endl;
    cout << endl;
    // --------------------
    
    
    preds = ldipsL3(examples, transitions, ops, sketch_depth, min_accuracy, aspPathBase+"out"+to_string(iteration)+"/");
}

void setupLdips(){
    Var x ("x", Dimension(1, 0, 0), NUM);
    Var v ("v", Dimension(1, -1, 0), NUM);
    Var decMax ("decMax", Dimension(1, -2, 0), NUM);
    Var vMax ("vMax", Dimension(1, -1, 0), NUM);
    Var zeroVel ("zeroVel", Dimension(1, -1, 0), NUM);
    Var target ("target", Dimension(1, 0, 0), NUM);

    variables.insert(x);
    variables.insert(v);
    variables.insert(decMax);
    variables.insert(vMax);
    variables.insert(zeroVel);
    variables.insert(target);

    // transitions.push_back(pair<string, string> ("ACC", "ACC"));
    transitions.push_back(pair<string, string> ("ACC", "DEC"));
    transitions.push_back(pair<string, string> ("ACC", "CON"));
    transitions.push_back(pair<string, string> ("DEC", "ACC"));
    // transitions.push_back(pair<string, string> ("DEC", "DEC"));
    transitions.push_back(pair<string, string> ("DEC", "CON"));
    transitions.push_back(pair<string, string> ("CON", "ACC"));
    transitions.push_back(pair<string, string> ("CON", "DEC"));
    // transitions.push_back(pair<string, string> ("CON", "CON"));
}


int main(int argc, char** argv){

    // Read known sequences from demonstration
    readData(obsDataPath, dataObs, dataLA);
    setupLdips();

    // Initialization
    library = ReadLibrary(operationLibPath);
    curASP_em = initialASP;

    // TO-DO: support passing in multiple different robots
    Robot jimmy_bot = Robot(5, -4, 12, 100, normal_distribution<double>(0, 1), 0.9);

    for(int i = 0; i < numIterations; i++){
        // Expectation
        cout << "Loop " << i << " expectation:" << endl;
        vector<Example> examples = expectation(&jimmy_bot, i);      // uses preds

        // Maximization
        cout << "Loop " << i << " maximization:" << endl;
        maximization(examples, i);     // updates preds, which is used by transitionUsingASPTree

        curASP_em = transitionUsingASPTree;
    }

    return 0;
}