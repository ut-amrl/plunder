
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
#include "visitors/print_visitor.hpp"
#include "ast/synthesis.hpp"
#include "../robot.h"
#include "../particleFilter/pf_runner.cpp"

using AST::ast_ptr;
using AST::BinOp;
using AST::BOOL;
using AST::Dimension;
using AST::Example;
using AST::Feature;
using AST::FunctionEntry;
using AST::Interpret;
using AST::Model;
using AST::Num;
using AST::NUM;
using AST::Param;
using AST::Signature;
using AST::Sketch;
using AST::SymEntry;
using AST::Type;
using AST::Var;
using AST::VEC;
using Eigen::Vector2f;
using std::cout;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::map;
using std::ofstream;
using std::ifstream;
using std::string;
using std::unordered_map;
using std::unordered_set;
using std::vector;
using json = nlohmann::json;
using z3::context;
using z3::solver;
using namespace std;


typedef HA asp_t(HA, Obs, Robot*);


const string obsDataPath = "pips/examples/data.json";
const string aspPath = "synthesis/out/";
const string hiLvlDataPath = "particleFilter/out/pf.csv";
const string operationLibPath = "pips/ops/test_library.json";

// LDIPS parameters
const int window_size = 0;
const int feature_depth = 3;
const int sketch_depth = 3;
const int min_accuracy = 0.2;

// PF parameters
const int pfN = 1000;
const int pfResampleThreshold = 0.2;

vector<Obs> dataObs;
vector<LA> dataLA;

unordered_set<Var> variables;
vector<std::pair<string,string>> transitions;
vector<ast_ptr> preds;
vector<FunctionEntry> library;

Robot globalRobot;


string haToString(HA ha){
    if(ha == ACC) return "ACC";
    if(ha == DEC) return "DEC";
    if(ha == CON) return "CON";
    return "CON";
}

HA stringToHA(string str){
    if(str == "ACC") return ACC;
    if(str == "DEC") return DEC;
    if(str == "CON") return CON;
    return CON;
}

Example dataToExample(Obs obs, HA curState){
    Example ex;
    SymEntry xEntry((float)obs.pos);
    SymEntry vEntry((float)obs.vel);
    SymEntry curStateEntry(haToString(curState));
    SymEntry accMaxEntry((float)globalRobot.accMax);
    SymEntry vMaxEntry((float)globalRobot.vMax);
    SymEntry targetEntry((float)globalRobot.target);
    ex.symbol_table_["x"] = xEntry;
    ex.symbol_table_["v"] = vEntry;
    ex.symbol_table_["start"] = curStateEntry;
    ex.symbol_table_["accMax"] = accMaxEntry;
    ex.symbol_table_["vMax"] = vMaxEntry;
    ex.symbol_table_["target"] = targetEntry;
    return ex;
}

Example dataToExample2(Obs obs, HA curState, HA nextState){
    Example ex = dataToExample(obs, curState);
    SymEntry nextStateEntry(haToString(nextState));
    ex.symbol_table_["output"] = nextStateEntry;
    return ex;
}

HA transitionUsingASPTree(HA curState, Obs obs, Robot* r){
    Example obsObject = dataToExample(obs, curState);
    for(uint i=0; i<transitions.size(); i++){
        if(haToString(curState) == transitions[i].first){
            if(InterpretBool(preds[i], obsObject)) return stringToHA(transitions[i].second);
        }
    }
    return CON;
}

HA initialASP(HA ha, Obs obs, Robot* r){
    return ACC;
}

vector<Example> expectation(){
    vector<HA> traj = runFilter(pfN, pfResampleThreshold, &globalRobot, dataObs, dataLA, curASP);
    vector<Example> examples;
    for(int i=0; i<dataObs.size()-1; i++){
        Example ex = dataToExample2(dataObs[i], traj[i], traj[i+1]);
        examples.push_back(ex);
    }
    return examples;
}

void maximization(vector<Example> examples){

    // vector<AST::Example> examples = ReadExamples(obsDataPath, variables, &transitions);

    // this is where we can optimize and remove the last transition of each if-statement
    // we can also order the transitions however we want

    std::reverse(transitions.begin(), transitions.end());
    examples = WindowExamples(examples, window_size);
    
    // Turn variables into roots
    vector<ast_ptr> inputs, roots;
    for (const Var& variable : variables) {
        roots.push_back(make_shared<Var>(variable));
    }

    vector<Signature> signatures;
    vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library, feature_depth, &signatures);
    
    preds = ldipsL3(examples, transitions, ops, sketch_depth, min_accuracy, aspPath);
}

void setupLdips(){
    Var x ("x", Dimension(1, 0, 0), NUM);
    Var v ("v", Dimension(1, -1, 0), NUM);
    Var accMax ("accMax", Dimension(1, -2, 0), NUM);
    Var vMax ("vMax", Dimension(1, -1, 0), NUM);
    Var target ("target", Dimension(1, 0, 0), NUM);
    Var start ("start", Dimension(1, 1, 0), AST::STATE);
    Var output ("output", Dimension(1, 1, 0), AST::STATE);
    variables.insert(x);
    variables.insert(v);
    variables.insert(accMax);
    variables.insert(vMax);
    variables.insert(target);
    variables.insert(start);
    variables.insert(output);
    pair<string, string> t1 ("ACC", "ACC");
    pair<string, string> t2 ("ACC", "DEC");
    pair<string, string> t3 ("ACC", "CON");
    pair<string, string> t4 ("DEC", "ACC");
    pair<string, string> t5 ("DEC", "DEC");
    pair<string, string> t6 ("DEC", "CON");
    pair<string, string> t7 ("CON", "ACC");
    pair<string, string> t8 ("CON", "DEC");
    pair<string, string> t9 ("CON", "CON");
    transitions.push_back(t1);
    transitions.push_back(t2);
    transitions.push_back(t3);
    transitions.push_back(t4);
    transitions.push_back(t5);
    transitions.push_back(t6);
    transitions.push_back(t7);
    transitions.push_back(t8);
    transitions.push_back(t9);
}


int main(int argc, char** argv){

    readData(obsDataPath, dataObs, dataLA);
    library = ReadLibrary(operationLibPath);
    setupLdips();
    asp_t* curASP = initialASP;

    for(int i=0; i<10; i++){
        cout << "loop " << i << " expectation" << endl;
        vector<Example> examples = expectation();      // uses preds

        cout << "loop " << i << " maximization" << endl;
        maximization(examples);     // updates preds, which is used by transitionUsingASPTree

        curASP = transitionUsingASPTree;
    }

    return 0;
}

