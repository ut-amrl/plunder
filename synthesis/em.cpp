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

// ignores the last 20 time steps of the particle filter because they are weird and introduce wrong transitions??
#define END_PF_ERR 20

using namespace std;
using namespace AST;
using namespace z3;
using Eigen::Vector2f;
using json = nlohmann::json;

unordered_set<Var> variables;
vector<pair<string,string>> transitions;
vector<float> accuracies;
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
    ex.symbol_table_["decMax"] = SymEntry((float) robot.decMax);
    ex.symbol_table_["vMax"] = SymEntry((float) robot.vMax);
    ex.symbol_table_["target"] = SymEntry((float) robot.target);

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
                break;
            }
        }
    }

    return pointError(ha, robot); // Introduce point errors - random transitions allow model to escape local minima
}

// Initial ASP: random transitions
HA initialASP(HA ha, Obs state, Robot& r){
    return pointError(ha, r);
}

// Expectation step
vector<vector<Example>> expectation(uint iteration, vector<Robot>& robots, vector<vector<Obs>>& dataObs, vector<vector<LA>>& dataLa, asp_t* asp){

    vector<vector<Example>> examples;

    for(uint i = 0; i < robots.size(); i++){
        string in = stateGenPath + to_string(i) + ".csv";
        string out = trajGenPath + to_string(iteration) + "-" + to_string(i) + ".csv";
        examples.push_back(vector<Example>());

        // Run filter
        vector<vector<HA>> trajectories = filterFromFile(numParticles, numTrajectories, resampleThreshold, robots[i], in, out, dataObs[i], dataLa[i], asp);
        
        // Convert each particle trajectory point to LDIPS-supported Example
        for(uint n = 0; n < sampleSize; n++){
            vector<HA> traj = trajectories[n];
            for(uint t = 0; t < dataObs[i].size() - 1 - END_PF_ERR; t++){
                Example ex = dataToExample(traj[t], dataObs[i][t+1], robots[i]);

                // Provide next high-level action
                ex.result_ = SymEntry(HAToString(traj[t+1]));
                examples[i].push_back(ex);
            }
        }
    }

    return examples;
}

// Maximization step
void maximization(vector<vector<Example>>& allExamples, uint iteration){
    vector<Example> examples;
    for(vector<Example>& each : allExamples){
        examples.insert(end(examples), begin(each), end(each));
    }

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

    cout << "---- Number of Features Enumerated ----" << endl;
    cout << ops.size() << endl << endl;
    cout << endl;

    cout << "Number of examples: " << examples.size() << endl;

    // Calculate new minimum accuracy
    // Cap each minimum accuracy to prevent iterating over the entire search space
    for(uint i = 0; i < transitions.size(); i++){
        if(accuracies.size() < transitions.size())
            accuracies.push_back(0);
        else
            accuracies[i] = min(accuracies[i], min_accuracy);
    }

    // Retrieve ASPs and accuracies    
    string aspFilePath = aspPathBase + to_string(iteration) + "/";
    filesystem::create_directory(aspFilePath);
    EmdipsOutput eo = emdips(examples, transitions, ops, sketch_depth, accuracies, aspFilePath);
    preds = eo.ast_vec;
    accuracies = eo.transition_accuracies;

    // Write ASP info to file
    ofstream aspStrFile;
    string aspStrFilePath = aspPathBase + to_string(iteration) + "/asp.txt";
    aspStrFile.open(aspStrFilePath);
    for(uint i = 0; i < transitions.size(); i++){
        aspStrFile << transitions[i].first + " -> " + transitions[i].second << endl;
        aspStrFile << "Accuracy: " << accuracies[i] << endl;
        aspStrFile << preds[i] << endl;
    }
    aspStrFile.close();
}

// Settings
void setupLdips(){
    Var x ("x", Dimension(1, 0, 0), NUM);
    Var v ("v", Dimension(1, -1, 0), NUM);
    Var decMax ("decMax", Dimension(1, -2, 0), NUM);
    Var vMax ("vMax", Dimension(1, -1, 0), NUM);
    Var target ("target", Dimension(1, 0, 0), NUM);

    variables.insert(x);
    variables.insert(v);
    variables.insert(decMax);
    variables.insert(vMax);
    variables.insert(target);

    for(uint i = 0; i < numHA; i++){
        for(uint j = 0; j < numHA; j++){
            if(i != j) transitions.push_back(pair<string, string> (HAToString(static_cast<HA>(i)), HAToString(static_cast<HA>(j))));
        }
    }
}

void testExampleOnASP(vector<Example> examples, Robot r){
    for(uint i=0; i<examples.size(); i++){
        Example e = examples[i];
        float x = e.symbol_table_["x"].GetFloat();
        float v = e.symbol_table_["v"].GetFloat();
        float decMax = e.symbol_table_["decMax"].GetFloat();
        float vMax = e.symbol_table_["vMax"].GetFloat();
        float target = e.symbol_table_["target"].GetFloat();
        string start = e.start_.GetString();
        string res = e.result_.GetString();
        if(start == "ACC") {
            string ASP_gen = ((r.DistTraveled(v, decMax) - r.DistTraveled(vMax, decMax) > -0.01) ? "CON" : "ACC");
            if(ASP_gen != res){
                cout << v << " " << vMax << " -> " << ASP_gen << " vs " << res << " : " << r.accMax << " " << r.decMax << endl;
            }
        }
    }
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
        if(useBoundaryError) setBoundaryStddev(boundaryDeviation); // use probabilistic ASP
        vector<vector<Example>> examples = expectation(i, robots, dataObs, dataLa, curASP);      // uses preds

        // Maximization
        cout << "Loop " << i << " maximization:" << endl;
        setBoundaryStddev(0); // reset boundaries during LDIPS
        maximization(examples, i);     // updates preds, which is used by transitionUsingASPTree

        curASP = ldipsASP;


        // Update point accuracy
        double satisfied = 0;
        double total = 0;
        for(uint r = 0; r < robots.size(); r++){
            // testExampleOnASP(examples[r], robots[r]);
            robots[r].pointAccuracy = 1; // make ASP deterministic
            for(Example& ex: examples[r]){
                total++;
                Obs obs = { .pos = ex.symbol_table_["x"].GetFloat(), .vel = ex.symbol_table_["v"].GetFloat() };
                if(curASP(stringToHA(ex.start_.GetString()), obs, robots[r]) == stringToHA(ex.result_.GetString())){
                    satisfied++;
                }
            }
        }
        
        // Update point accuracy
        double newPointAcc = min(satisfied / total, 0.95);
        cout << "New point accuracy: " << satisfied << " / " << total << " ~= " << newPointAcc << endl;
        for(Robot& r : robots){
            r.pointAccuracy = newPointAcc;
        }
    }
}

int main() {
    vector<Robot> robots = getRobotSet(robotTestSet, normal_distribution<double>(meanError, stddevError), pointAccuracy);
    emLoop(robots);

    return 0;
}