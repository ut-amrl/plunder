
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


const string obsDataPath = "pips/examples/data.json";
const string aspPath = "synthesis/out/";
const string hiLvlDataPath = "particleFilter/out/pf.csv";
const string operationLibPath = "pips/ops/test_library.json";

const int window_size = 0;
const int feature_depth = 3;
const int sketch_depth = 3;
const int min_accuracy = 0.2;


vector<std::pair<string,string>> transitions;
vector<ast_ptr> preds;



void collectDemos(){
    
}


Example obsToExample(Obs obs){
    Example ex;
    SymEntry xEntry(obs.pos);
    SymEntry vEntry(obs.vel);
    string stateStr = 
    SymEntry curStateEntry()
    ex.symbol_table_["x"] = xEntry;
    ex.symbol_table_["y"] = vEntry;
    ex.symbol_table_["state"] = curStateEntry;
    return ex;
}



string transitionUsingASP(string curState, Obs obs){
    Example obsObject = obsToExample(obs);
    for(uint i=0; i<transitions.size(); i++){
        if(curState == transitions[i].first){
            if(InterpretBool(preds[i], obsObject)) return transitions[i].second;
        }
    }
    return "";
}


void expectation(){

    unordered_set<Var> variables;
    vector<AST::Example> examples = ReadExamples(obsDataPath, variables, &transitions);

    // this is where we can optimize and remove the last transition of each if-statement

    std::reverse(transitions.begin(), transitions.end());
    examples = WindowExamples(examples, window_size);
    
    // Turn variables into roots
    vector<ast_ptr> inputs, roots;
    for (const Var& variable : variables) {
        roots.push_back(make_shared<Var>(variable));
    }

    // Get library
    vector<FunctionEntry> library = ReadLibrary(operationLibPath);

    vector<Signature> signatures;

    // Enumerate all features
    vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library, feature_depth, &signatures);

    preds = ldipsL3(examples, transitions, ops, sketch_depth, min_accuracy, aspPath);
    // for(uint i=0; i<transitions.size(); i++){
    //     cout << transitions[i].first << " -> " << transitions[i].second << endl;
    //     cout << preds[i] << endl;
    // }
}


void maximization(){
    HA state = ACC;     // initial state
    for(int i=0; i<10; i++){
        cout << transitionUsingASP(state, obs) << endl;
    }
}


int main(int argc, char** argv){

    for(int i=0; i<10; i++){
        cout << "loop " << i << " expectation" << endl;
        expectation();
        cout << "loop " << i << " maximization" << endl;
        maximization();
    }

    return 0;
}


