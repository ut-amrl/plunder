#include <dlfcn.h>
#include <gflags/gflags.h>
#include <z3++.h>
#include <eigen3/Eigen/Core>


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
// #include "sketches/sketches.hpp"

DEFINE_string(ex_file, "examples/data.json", "Examples file");
DEFINE_string(lib_file, "ops/emdips_test.json", "Operation library file");
DEFINE_string(out_dir, "ref/dipsl3/", "Operation library file");
DEFINE_string(feature_file, "features.txt", "File to write features to");
DEFINE_uint32(feat_depth, 3, "Maximum enumeration depth for features.");
DEFINE_uint32(sketch_depth, 2, "Maximum enumeration depth for sketch.");
DEFINE_uint32(window_size, 3, "Size of sliding window to subsample demonstrations with.");
DEFINE_double(target_score, 10, "What log likelihood should be achieved / what proportion of examples should be SAT to declare victory?");
DEFINE_bool(write_features, false, "Write all enumerated features to a file");
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
DEFINE_bool(debug, true, "Enable Debug Printing");
DEFINE_uint32(batch_size, 8, "Number of sketches to solve in one python call");

using namespace AST;
using namespace std;
using Eigen::Vector2f;
using json = nlohmann::json;
using z3::context;
using z3::solver;

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  if (FLAGS_target_score < 0.0)
    FLAGS_target_score = 0.0;

  // Read in the Examples
  // Also obtains variables that can be used (roots for synthesis)
  // and the possible input->output state pairs.
  unordered_set<Var> variables;
  vector<pair<string,string>> transitions;
  vector<Example> examples = ReadExamples(FLAGS_ex_file,
      variables,
      &transitions);

  // Sort transitions
  sort(transitions.begin(), transitions.end(), [](const pair<string, string>& a, const pair<string, string>& b) -> bool {
    if(a.first == b.first){
        if(a.first == a.second) return 1;
        if(b.first == b.second) return -1;
        return a.second < b.second;
    }
    return a.first < b.first;
  });

  examples = WindowExamples(examples, FLAGS_window_size);

  // Turning variables into roots
  vector<ast_ptr> inputs, roots;
  for (const Var& variable : variables) {
    if (variable.name_ != "goal" && variable.name_ != "free_path" && variable.name_ != "DoorState") {
      roots.push_back(make_shared<Var>(variable));
    }
  }
  
  vector<float> min_accuracies;
  for (auto& trans : transitions) {
    min_accuracies.push_back(FLAGS_target_score);
  }

  cout << "----Roots----" << endl;
  for (auto& node : roots) {
    cout << node << endl;
  }
  cout << endl;

  cout << "----Transitions----" << endl;
  for (int i = 0; i < transitions.size(); i++) {
    cout << transitions[i].first << "->" << transitions[i].second << endl;
  }
  cout << endl;

  // Loading Library Function Definitions
  vector<FunctionEntry> library = ReadLibrary(FLAGS_lib_file);

  cout << "----Library----" << endl;
  for (auto& func : library) {
    cout << func << endl;
  }
  cout << endl;

  // Enumerate features up to a fixed depth
  vector<Signature> signatures;
  vector<ast_ptr> ops = AST::RecEnumerateLogistic(roots, inputs, examples, library,
                                          FLAGS_feat_depth, &signatures);

  if (FLAGS_debug) {
      cout << "---- Features Synthesized ----" << endl;
      for (auto& feat : ops) {
          cout << feat << endl;
      }
      cout << endl;
  }

  cout << "---- Number of Features Enumerated ----" << endl;
  cout << ops.size() << endl << endl;
  cout << endl;

  // Run L3 Synthesis
  emdipsL3(examples, transitions, ops, FLAGS_sketch_depth, min_accuracies,
      FLAGS_out_dir, FLAGS_batch_size);
}
