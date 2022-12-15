// OUTDATED

// #include <dlfcn.h>
// #include <z3++.h>
// #include <assert.h>

// #include <fstream>
// #include <iomanip>
// #include <iostream>
// #include <filesystem>
// #include <memory>
// #include <algorithm>
// #include <unordered_map>
// #include <unordered_set>

// #include "ast/ast.hpp"
// #include "ast/enumeration.hpp"
// #include "ast/library_functions.hpp"
// #include "ast/parsing.hpp"
// #include "visitors/interp_visitor.hpp"
// #include "ast/synthesis.hpp"

// #include "../../particleFilter/pf_runner.h"
// #include "../../settings.h"

// using namespace std;
// using namespace AST;
// using namespace z3;
// using Eigen::Vector2f;
// using json = nlohmann::json;

// namespace std {
//     ostream& operator<<(ostream& os, const AST::ast_ptr& ast);
// }

// unordered_set<Var> variables;
// vector<pair<string,string>> transitions;
// vector<FunctionEntry> library;

// // Convert transition to LDIPS-compatible Example
// Example dataToExample(HA ha, Obs state, Robot& robot){
//     Example ex;

//     ex.symbol_table_["x"] = SymEntry((float) state.pos);
//     ex.symbol_table_["v"] = SymEntry((float) state.vel);
//     ex.symbol_table_["decMax"] = SymEntry((float) robot.decMax);
//     ex.symbol_table_["vMax"] = SymEntry((float) robot.vMax);
//     ex.symbol_table_["target"] = SymEntry((float) robot.target);

//     ex.start_ = SymEntry(to_string(ha));

//     return ex;
// }

// // Settings
// void setupLdips(){
//     Var x ("x", Dimension(1, 0, 0), NUM);
//     Var v ("v", Dimension(1, -1, 0), NUM);
//     Var decMax ("decMax", Dimension(1, -2, 0), NUM);
//     Var vMax ("vMax", Dimension(1, -1, 0), NUM);
//     Var target ("target", Dimension(1, 0, 0), NUM);

//     variables.insert(x);
//     variables.insert(v);
//     variables.insert(decMax);
//     variables.insert(vMax);
//     variables.insert(target);

//     for(uint i = 0; i < numHA; i++){
//         for(uint j = 0; j < numHA; j++){
//             if(i != j) transitions.push_back(pair<string, string> (to_string(to_label(i)), to_string(to_label(j))));
//         }
//     }
// }

// // Runs full LDIPS-generated ASP
// HA ldipsASP(HA ha, Obs state, Robot& robot, vector<ast_ptr> preds){
//     HA prevHA = ha;
//     Example obsObject = dataToExample(ha, state, robot);
//     for(uint i = 0; i < transitions.size(); i++){
//         if(to_string(ha) == transitions[i].first){
//             if(InterpretBool(preds[i], obsObject)) {
//                 ha = to_label(transitions[i].second);
//                 break;
//             }
//         }
//     }

//     return ha;
// }

// void probabilisticProgramTest(){
//     Robot r(5.0, -5.0, 20.0, 100.0, normal_distribution<double>(0.0, 2.0), 1.0);
//     vector<ast_ptr> preds(transitions.size());
//     ast_ptr fa = AstFromJson(json::parse("{\"dim\":[0,0,0],\"node\":\"Bool\",\"symbolic\":false,\"type\":5,\"value\":false}"));

//     json j = json::parse("\
//         {\
//             \"dim\": [ 0, 0, 0 ],\
//             \"left\": {\
//                 \"dim\": [ 0, 0, 0 ],\
//                 \"name\": \"fX1\",\
//                 \"node\": \"Feature\",\
//                 \"symbolic\": false,\
//                 \"type\": 2,\
//                 \"value\": {\
//                     \"dim\": [ 1, 0, 0 ],\
//                     \"name\": \"x\",\
//                     \"node\": \"Var\",\
//                     \"symbolic\": false,\
//                     \"type\": 2\
//                 }\
//             },\
//             \"node\": \"BinOp\",\
//             \"op\": \"Gt\",\
//             \"right\": {\
//                 \"dim\": [ 0, 0, 0 ],\
//                 \"name\": \"pX1\",\
//                 \"node\": \"Param\",\
//                 \"symbolic\": true,\
//                 \"type\": 2,\
//                 \"value\": {\
//                     \"dim\": [ 0, 0, 0 ],\
//                     \"node\": \"Num\",\
//                     \"symbolic\": false,\
//                     \"type\": 2,\
//                     \"value\": 10.0\
//                 }\
//             },\
//             \"symbolic\": false,\
//             \"type\": 4\
//         }\
//     ");

//     ast_ptr predArr[] = { fa, AstFromJson(j), fa, fa, fa, fa }; // ACC -> DEC, ACC -> CON, DEC -> ACC, DEC -> CON, CON -> ACC, CON -> DEC
 
//     preds.assign(begin(predArr), end(predArr));

//     HA ha = ACC;
//     Obs state = { .pos = 5.0, .vel = 15.0 };

//     int accCount = 0;
//     int conCount = 0;

//     for(int i = 0; i < 1000; i++){
//         HA newHa = ldipsASP(ha, state, r, preds);
//         if(newHa == to_label("ACC")){
//             accCount++;
//         } else if (newHa == to_label("CON")){
//             conCount++;
//         }
//     }
    
//     cout << "At position=" << state.pos << ": " << to_string(ha) << " -> ACC has probability " << accCount/10.0 << "%" << endl;
//     cout << "At position=" << state.pos << ": " << to_string(ha) << " -> CON has probability " << conCount/10.0 << "%" << endl;
// }

// int main() {
    
//     // Initialization
//     setupLdips();
//     library = ReadLibrary(operationLibPath);
//     setBoundaryStddev(boundaryDeviation);

//     probabilisticProgramTest();

//     return 0;
// }