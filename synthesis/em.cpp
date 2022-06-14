
#include "ast/synthesis.hpp"
#include "pf_runner.hpp"


using namespace std;


static const string obsDataPath = "accSim/out/data.json";
static const string aspPath = "synthesis/out/asp.json";
static const string hiLvlDataPath = "pf_custom/out/pf.csv";

void expectation(){

    // unordered_set<Var> variables;


    // examples
    // transitions
    // ops
    // sketch_depth
    // min_accuracy
    // out_dir


    // AST::ldipsL3();
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


