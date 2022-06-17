#include <dlfcn.h>
#include <z3++.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <filesystem>
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

#include "../particleFilter/pf_runner.h"
#include "../settings.h"

using namespace std;
using namespace AST;
using namespace z3;
using Eigen::Vector2f;
using json = nlohmann::json;

unordered_set<Var> variables;
vector<pair<string,string>> transitions;
vector<ast_ptr> preds;
vector<FunctionEntry> library;

namespace std {
    ostream& operator<<(ostream& os, const AST::ast_ptr& ast);
}

// Convert transition to LDIPS-compatible Example
Example dataToExample(HA ha, Obs state, Robot& robot){
    Example ex;

    ex.symbol_table_["x"] = SymEntry((float) state.pos);
    ex.symbol_table_["v"] = SymEntry((float) state.vel);
    ex.symbol_table_["decMax"] = ((float) robot.decMax);
    ex.symbol_table_["vMax"] = ((float) robot.vMax);
    ex.symbol_table_["target"] = ((float) robot.target);
    ex.symbol_table_["zeroVel"] = ((float) 0);
    ex.start_ = SymEntry(HAToString(ha));

    return ex;
}

// Runs LDIPS-generated ASP
HA ldipsASP(HA ha, Obs state, Robot& robot){
    Example obsObject = dataToExample(ha, state, robot);
    for(uint i = 0; i < transitions.size(); i++){
        if(HAToString(ha) == transitions[i].first){
            if(InterpretBool(preds[i], obsObject)) {
                ha = stringToHA(transitions[i].second);
                return putErrorIntoHA(ha, robot);
            }
        }
    }

    return putErrorIntoHA(ha, robot);
}

// Initial ASP: random transitions
HA initialASP(HA ha, Obs state, Robot& r){
    return ASP_random(ha, state, r);
}

// Expectation step
vector<Example> expectation(uint iteration, vector<Robot>& robots, vector<vector<Obs>>& dataObs, vector<vector<LA>>& dataLa, asp_t* asp){
    vector<Example> examples;

    for(uint i = 0; i < robots.size(); i++){
        string in = stateGenPath + to_string(i) + ".csv";
        string out = trajGenPath + to_string(iteration) + "-" + to_string(i) + ".csv";
        
        // Run filter
        vector<vector<HA>> trajectories = filterFromFile(numParticles, resampleThreshold, robots[i], in, out, dataObs[i], dataLa[i], asp);
        
        // Convert each particle trajectory point to LDIPS-supported Example
        for(uint n = 0; n < numParticles; n++){
            vector<HA> traj = trajectories[n];
            for(uint t = 0; t < dataObs[i].size() - 1; t++){
                Example ex = dataToExample(traj[t], dataObs[i][t], robots[i]);

                // Provide next high-level action
                ex.result_ = SymEntry(HAToString(traj[t+1]));
                examples.push_back(ex);
            }
        }
    }

    return examples;
}

// Maximization step
void maximization(vector<Example>& examples, uint iteration){
    
    examples = WindowExamples(examples, window_size);
    
    // Turn variables into roots
    vector<ast_ptr> inputs, roots;
    for (const Var& variable : variables) {
        roots.push_back(make_shared<Var>(variable));
    }

    vector<Signature> signatures;
    vector<ast_ptr> ops = RecEnumerate(roots, inputs, examples, library, feature_depth, &signatures);

    // Debug
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

    cout << "Number of examples: " << examples.size() << endl;

    // Write ASPs to directory    
    string aspFilePath = aspPathBase + to_string(iteration) + "/";
    filesystem::create_directory(aspFilePath);
    preds = ldipsL3(examples, transitions, ops, sketch_depth, min_accuracy, aspFilePath);
}

// Settings
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

    transitions.push_back(pair<string, string> ("ACC", "DEC"));
    transitions.push_back(pair<string, string> ("ACC", "CON"));
    transitions.push_back(pair<string, string> ("DEC", "ACC"));
    transitions.push_back(pair<string, string> ("DEC", "CON"));
    transitions.push_back(pair<string, string> ("CON", "ACC"));
    transitions.push_back(pair<string, string> ("CON", "DEC"));
    // else statements
    // transitions.push_back(pair<string, string> ("ACC", "ACC"));
    // transitions.push_back(pair<string, string> ("DEC", "DEC"));
    // transitions.push_back(pair<string, string> ("CON", "CON"));
}


void emLoop(vector<Robot>& robots){

    // Initialization
    setupLdips();
    library = ReadLibrary(operationLibPath);
    asp_t* curASP = initialASP;
    vector<vector<Obs>> dataObs (robots.size());
    vector<vector<LA>> dataLa (robots.size());

    for(int i = 0; i < numIterations; i++){
        
        // Expectation
        cout << "Loop " << i << " expectation:" << endl;
        vector<Example> examples = expectation(i, robots, dataObs, dataLa, curASP);      // uses preds

        // Maximization
        cout << "Loop " << i << " maximization:" << endl;
        maximization(examples, i);     // updates preds, which is used by transitionUsingASPTree

        curASP = ldipsASP;
    }
}

int main() {
    vector<Robot> robots = getRobotSet(robotTestSet, normal_distribution<double>(meanError, stddevError), haProbCorrect);
    emLoop(robots);

    return 0;
}