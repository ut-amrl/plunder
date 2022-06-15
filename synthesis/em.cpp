
#include <dlfcn.h>
#include <gflags/gflags.h>
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

using AST::ast_ptr;
using AST::BinOp;
using AST::BOOL;
using AST::Dimension;
using AST::Example;
using AST::Feature;
using AST::FunctionEntry;
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


const string obsDataPath = "accSim/out/data.json";
const string aspPath = "synthesis/out/";
const string hiLvlDataPath = "pf_custom/out/pf.csv";
const string operationLibPath = "pips/ops/test_library.json";

const int window_size = 10;
const int feature_depth = 2;
const int sketch_depth = 2;
const int min_accuracy = 0.8;


void collectDemos(){
    
}

void expectation(){

    unordered_set<Var> variables;
    vector<std::pair<string,string>> transitions;
    vector<AST::Example> examples = ReadExamples(obsDataPath, variables, &transitions);

    std::reverse(transitions.begin(), transitions.end());
    examples = WindowExamples(examples, window_size);
    
    // Turn variables into roots
    vector<ast_ptr> roots;
    for (const Var& variable : variables) {
        if (variable.name_ != "goal" && variable.name_ != "free_path" && variable.name_ != "DoorState") {
            roots.push_back(make_shared<Var>(variable));
        }
    }

    // Get library
    vector<FunctionEntry> library = ReadLibrary(operationLibPath);

    vector<ast_ptr> inputs;
    vector<Signature> signatures;

    // Enumerate all features
    vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library, feature_depth, &signatures);


    ldipsL3(examples, transitions, ops, sketch_depth, min_accuracy, aspPath);
}


void maximization(){

    // Robot robotModel;
    // pf.run(obsDataPath, hiLvlDataPath, robotModel);

}


int main(int argc, char** argv){

    for(int i=0; i<10; i++){
        expectation();
        maximization();
    }


    return 0;
}


