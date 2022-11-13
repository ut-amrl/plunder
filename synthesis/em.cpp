#include <dlfcn.h>
#include <z3++.h>
#include "Python.h"

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

#include "particleFilter/pf_runner.h"
#include "settings.h"
#include "accSim/generate.h"

using namespace std;
using namespace AST;
using Eigen::Vector2f;
using json = nlohmann::json;

unordered_set<Var> variables;
vector<pair<string,string>> transitions;
vector<FunctionEntry> library;
vector<ast_ptr> roots;
PyObject* pFunc;

vector<float> accuracies;
vector<ast_ptr> preds;
vector<ast_ptr> gt_truth;

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

float distt(float v, float d){
    return - v * v / (2 * d);
}

void printExampleInfo(Example e){
    float x = e.symbol_table_["x"].GetFloat();
    float v = e.symbol_table_["v"].GetFloat();
    float decMax = e.symbol_table_["decMax"].GetFloat();
    float vMax = e.symbol_table_["vMax"].GetFloat();
    float target = e.symbol_table_["target"].GetFloat();
    string start = e.start_.GetString();
    string res = e.result_.GetString();
    float exp = x + distt(v, decMax);
    cout << start << "->" << res << ", x " << x << ", v " << v << ", decMax " << decMax << ", vMax " << vMax << ", target " << target << ", exp " << exp << endl;
}

// Runs LDIPS-generated ASP
HA ldipsASP(HA ha, Obs state, Robot& robot){
    HA prevHA = ha;
    Example obsObject = dataToExample(ha, state, robot);
    for(uint i = 0; i < transitions.size(); i++){
        // cout << "testing transition " << transitions[i].first << " -> " << transitions[i].second << endl;

        if(HAToString(prevHA) == transitions[i].first && transitions[i].first!=transitions[i].second){
            if(InterpretBool(preds[i], obsObject)) {
                ha = stringToHA(transitions[i].second);
                break;
            }
        }
    }

    return pointError(prevHA, ha, robot, false); // Introduce point errors - random transitions allow model to escape local minima
}

// Initial ASP: random transitions
HA initialASP(HA ha, Obs state, Robot& r){
    // return ASP_random(ha, state, r);
    return pointError(ha, ha, r, false);
}

bool isValidExample(Example ex){
    // return (ex.start_.GetString()==("ACC") && ex.result_.GetString()==("CON")) ||
    //         (ex.start_.GetString()==("ACC") && ex.result_.GetString()==("ACC")) ||
    //         (ex.start_.GetString()==("ACC") && ex.result_.GetString()==("DEC")) ||
    //         (ex.start_.GetString()==("CON") && ex.result_.GetString()==("DEC")) ||
    //         (ex.start_.GetString()==("CON") && ex.result_.GetString()==("CON")) ||
    //         (ex.start_.GetString()==("DEC") && ex.result_.GetString()==("DEC"));
    return true;
}

// Expectation step
vector<vector<Example>> expectation(uint iteration, vector<Robot>& robots, vector<vector<Obs>>& dataObs, vector<vector<LA>>& dataLa, asp_t* asp){

    vector<vector<Example>> examples;

    cout << "Running particle filter with " << numParticles << " particles\n";
    cout << "Parameters: resample threshold=" << resampleThreshold << ", observation strength=" << obsLikelihoodStrength << endl;

    double cum_log_obs = 0;
    for(uint i = 0; i < numRobots; i++){
        string in = stateGenPath + to_string(i) + ".csv";
        string out = trajGenPath + to_string(iteration) + "-" + to_string(i) + ".csv";
        examples.push_back(vector<Example>());

        // Run filter
        vector<vector<HA>> trajectories;
        cum_log_obs += filterFromFile(trajectories, numParticles, numTrajectories, resampleThreshold, robots[i], in, out, dataObs[i], dataLa[i], asp);


        shuffle(begin(trajectories), end(trajectories), default_random_engine {});
        
        // Convert each particle trajectory point to LDIPS-supported Example
        for(uint n = 0; n < sampleSize; n++){
            vector<HA> traj = trajectories[n];
            for(uint t = 0; t < dataObs[i].size() - 1 - end_pf_err; t++){
                Example ex = dataToExample(traj[t], dataObs[i][t+1], robots[i]);

                // Provide next high-level action
                ex.result_ = SymEntry(HAToString(traj[t+1]));
                if(isValidExample(ex)) examples[i].push_back(ex);
            }
        }

        // Run ASPs for all robots
        string s = altPath+to_string(iteration)+"-"+to_string(i)+".csv";
        executeASP(robots[i], s, dataObs[i], asp);

        cout << "*";
        cout.flush();
    }

    cout << "\r";
    cout << "Cumulative observation likelihood: e^" << cum_log_obs << " = " << exp(cum_log_obs) << endl;

    return examples;
}

void sampleFromExamples(vector<vector<Example>>& allExamples, vector<Example>& sampleOfExamples, vector<Example>& consolidated){
    for(vector<Example>& each : allExamples){
        consolidated.insert(end(consolidated), begin(each), end(each));
    }

    consolidated = WindowExamples(consolidated, window_size);
    shuffle(begin(consolidated), end(consolidated), default_random_engine {});

    std::sort(consolidated.begin(), consolidated.end(), [](const Example& a, const Example& b) -> bool {
        if(a.start_.GetString() == b.start_.GetString()){
            if(a.start_.GetString() == a.result_.GetString()) return false;
            if(b.start_.GetString() == b.result_.GetString()) return true;
            return a.result_.GetString() < b.result_.GetString();
        }
        return a.start_.GetString() < b.start_.GetString();
    });

    int count = 0;
    for(int i = 0; i < consolidated.size(); i++) {
        if(i != 0 && (consolidated[i].start_.GetString() != consolidated[i-1].start_.GetString() || 
                        consolidated[i].result_.GetString() != consolidated[i-1].result_.GetString())) {
            count = 0;
        } else {
            if(count < max_examples) {
                count++;
                sampleOfExamples.push_back(consolidated[i]);
            }
        }
    }
}

void sample2(vector<vector<Example>>& allExamples, vector<Example>& sampleOfExamples){
    for(uint r=0; r<numRobots; r++){
        uint trajLength = allExamples[r].size()/sampleSize;
        for(uint i=0; i<sampleSize; i++){
            uint last_transition_t=0;
            for(uint t=window_size/2; t<trajLength-window_size/2-1; t++){
                Example e = allExamples[r][i*trajLength+t];
                string expected_start = e.start_.GetString();
                string expected_end = e.result_.GetString();
                if(expected_start != expected_end){
                    // CAPTURE MIDDLE BETWEEN THIS AND PREV TRANSITION - ADD CONSISTENCY
                    if(t-last_transition_t>1){
                        uint mid_t = (t+last_transition_t)/2;
                        Example e_new = allExamples[r][i*trajLength+mid_t];
                        if(e_new.start_.GetString()==expected_start && e_new.result_.GetString()==expected_start){
                            sampleOfExamples.push_back(e_new);
                        }
                    }
                    // LEFT OF TRANSITION
                    for(uint w=1; w<=window_size/2; w+=1){
                        Example e_new = allExamples[r][i*trajLength+t-w];
                        if(e_new.start_.GetString()==expected_start && e_new.result_.GetString()==expected_start){
                            sampleOfExamples.push_back(e_new);
                        } else break;
                    }
                    // MIDDLE OF TRANSITION
                    {
                        Example e_new = allExamples[r][i*trajLength+t];
                        if(e_new.start_.GetString()==expected_start && e_new.result_.GetString()==expected_end){
                            sampleOfExamples.push_back(e_new);
                        }
                    }
                    // RIGHT OF TRANSITION
                    for(uint w=1; w<=window_size/2; w+=1){
                        Example e_new = allExamples[r][i*trajLength+t+w];
                        if(e_new.start_.GetString()==expected_end && e_new.result_.GetString()==expected_end){
                            e_new.start_ = e.start_;
                            sampleOfExamples.push_back(e_new);
                        } else break;
                    }
                    last_transition_t=t;
                }
            }
            // MIDDLE BETNWEEN LAST TRANSITION AND ENDING
            {
                uint t=trajLength-1;
                if(t-last_transition_t>1){
                    uint mid_t = (t+last_transition_t)/2;
                    Example e_new = allExamples[r][i*trajLength+mid_t];
                    sampleOfExamples.push_back(e_new);
                }
            }
        }
    }
}

// Maximization step
void maximization(vector<vector<Example>>& allExamples, uint iteration){
    
    vector<Example> sampleOfExamples;
    vector<Example> consolidated;
    sampleFromExamples(allExamples, sampleOfExamples, consolidated);
    cout << "Number of examples: sampled " << sampleOfExamples.size() << " examples out of " << consolidated.size() << " total\n";

    
    // vector<Example> sampleOfExamples;
    // sample2(allExamples, sampleOfExamples);


    // for(Example e: sampleOfExamples){
    //     printExampleInfo(e);
    // }

    // Set each maximum error to speed up search
    cout << "Setting error threshold to " << max_error << "\n\n";
    for(uint i = 0; i < transitions.size(); i++){
        accuracies[i] = max_error;
    }
    
    EmdipsOutput eo;
    if(iteration % structuralChangeFrequency == 0 && !hardcode_program){

        vector<ast_ptr> inputs; vector<Signature> sigs;
        vector<ast_ptr> ops = AST::RecEnumerateLogistic(roots, inputs, sampleOfExamples, library,
                                            feature_depth, &sigs);

        cout << "---- Number of Features Enumerated ----" << endl;
        cout << ops.size() << endl << endl;
        for(int i = 0; i < 5; i++){
            cout << ops[i] << endl;
        }
        cout << "...\n\n\n";

        // Retrieve ASPs and accuracies    
        string aspFilePath = aspPathBase + to_string(iteration) + "/";
        filesystem::create_directory(aspFilePath);

        vector<ast_ptr> all_sketches = EnumerateL3(ops, sketch_depth);
        
        cout << "---- Number of Total Programs ----" << endl;
        cout << all_sketches.size() << endl;
        // for(ast_ptr each: all_sketches){
        //     cout << each << endl;
        // }

        eo = emdipsL3(sampleOfExamples, transitions, all_sketches, preds, gt_truth, accuracies, aspFilePath, batch_size, programs_enumerated, false, pFunc);

    } else {
        
        // Retrieve ASPs and accuracies    
        string aspFilePath = aspPathBase + to_string(iteration) + "/";
        filesystem::create_directory(aspFilePath);
        vector<ast_ptr> all_sketches;
        eo = emdipsL3(sampleOfExamples, transitions, all_sketches, preds, gt_truth, accuracies, aspFilePath, batch_size, programs_enumerated, true, pFunc);

    }

    
    preds = eo.ast_vec;
    accuracies = eo.log_likelihoods;

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
    cout << "-------------Setup----------------" << endl;

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
            transitions.push_back(pair<string, string> (HAToString(static_cast<HA>(i)), HAToString(static_cast<HA>(j))));
            accuracies.push_back(numeric_limits<float>::max());
        }
    }
    
    std::sort(transitions.begin(), transitions.end(), [](const pair<string, string>& a, const pair<string, string>& b) -> bool {
        if(a.first == b.first){
            if(a.first == a.second) return false;
            if(b.first == b.second) return true;
            return a.second < b.second;
        }
        return a.first < b.first;
    });

    // Turn variables into roots
    for (const Var& variable : variables) {
        roots.push_back(make_shared<Var>(variable));
    }

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

    if(hardcode_program){
        cout << "----Using fixed program----" << endl;
    } else {
        cout << "----Ground truth (target) program----" << endl;
    }
    
    // Read hard coded program structure
    for (int t = 0; t < transitions.size(); t++) {
        const auto &transition = transitions[t];
        const string input_name =
            gt_asp + transition.first + "_" + transition.second + ".json";

        ifstream input_file;
        input_file.open(input_name);
        const json input = json::parse(input_file);
        ast_ptr fixed = AstFromJson(input);
        gt_truth.push_back(fixed);

        cout << transition.first << " -> " << transition.second << ": " << fixed << endl;
        
        input_file.close();
    }
}


void testExampleOnASP(vector<Example> examples, Robot r){
    for(uint i=0; i<examples.size(); i++){
        Example e = examples[i];
        printExampleInfo(e);
    }
}

void emLoop(vector<Robot>& robots){

    // Initialization
    setupLdips();

    library = ReadLibrary(operationLibPath);
    asp_t* curASP = initialASP;
    vector<vector<Obs>> dataObs (numRobots);
    vector<vector<LA>> dataLa (numRobots);

    for(int r = 0; r < numRobots; r++){
        // Run ground truth ASP

        string inputFile = stateGenPath + to_string(r) + ".csv";
        readData(inputFile, dataObs[r], dataLa[r]);
        string s = altPath+"gt-"+to_string(r)+".csv";
        executeASP(robots[r], s, dataObs[r], ASP_model(model));
    }

    for(int i = 0; i < numIterations; i++){
        
        // Expectation
        cout << "\n|-------------------------------------|\n";
        cout << "|                                     |\n";
        cout << "|          Loop " << i << " EXPECTATION         |\n";
        cout << "|                                     |\n";
        cout << "|-------------------------------------|\n";
        vector<vector<Example>> examples = expectation(i, robots, dataObs, dataLa, curASP);      // uses preds

        // Maximization
        cout << "\n|-------------------------------------|\n";
        cout << "|                                     |\n";
        cout << "|         Loop " << i << " MAXIMIZATION         |\n";
        cout << "|                                     |\n";
        cout << "|-------------------------------------|\n";
        maximization(examples, i);     // updates preds, which is used by transitionUsingASPTree

        curASP = ldipsASP;


        // // Update point accuracy
        // double satisfied = 0;
        // double total = 0;
        // for(uint r = 0; r < numRobots; r++){
        //     // testExampleOnASP(examples[r], robots[r]);
        //     robots[r].pointAccuracy = 1; // make ASP deterministic
        //     for(Example& ex: examples[r]){
        //         total++;
        //         Obs obs = { .pos = ex.symbol_table_["x"].GetFloat(), .vel = ex.symbol_table_["v"].GetFloat() };
        //         if(curASP(stringToHA(ex.start_.GetString()), obs, robots[r]) == stringToHA(ex.result_.GetString())){
        //             satisfied++;
        //         }
        //     }
        // }
        
        // // Update point accuracy
        // double newPointAcc = min(satisfied / total, 0.95);
        // cout << "New point accuracy: " << satisfied << " / " << total << " ~= " << newPointAcc << endl;
        // for(Robot& r : robots){
        //     r.pointAccuracy = newPointAcc;
        // }
    }
}

int main() {
    // Set up Python
    // Initialize python support
    Py_Initialize();

    PyRun_SimpleString(
        "import os, sys \n"
        "sys.path.append(os.getcwd() + '/pips/src/optimizer') \n");

    // File name
    PyObject* pName = PyUnicode_FromString((char*)"optimizer");

    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        // Function name
        pFunc = PyObject_GetAttrString(pModule, (char*)"run_optimizer_threads");

        if (!(pFunc && PyCallable_Check(pFunc))) {
            if (PyErr_Occurred()) PyErr_Print();
            fprintf(stderr, "Cannot find optimization function\n");
        }
    } else {
        PyErr_Print();
        fprintf(stderr, "Failed to load optimization file");
    }

    vector<Robot> robots = getRobotSet(robotTestSet, normal_distribution<double>(meanError, stddevError), pointAccuracy);
    emLoop(robots);

    // Clean up python
    Py_XDECREF(pFunc);
    Py_DECREF(pModule);
    Py_Finalize();

    return 0;
}
