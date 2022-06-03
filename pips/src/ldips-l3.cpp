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
// #include "sketches/sketches.hpp"

DEFINE_string(ex_file, "", "Examples file");
DEFINE_string(lib_file, "ops/social_test.json", "Operation library file");
DEFINE_string(out_dir, "ref/dipsl3/", "Operation library file");
DEFINE_uint32(feat_depth, 3, "Maximum enumeration depth for features.");
DEFINE_uint32(sketch_depth, 3, "Maximum enumeration depth for sketch.");
DEFINE_uint32(window_size, 3, "Size of sliding window to subsample demonstrations with.");
DEFINE_double(min_accuracy, 1.0,
              "What proportion of examples should be SAT to declare victory?");
DEFINE_bool(write_features, false, "Write all enumerated features to a file");
DEFINE_string(feature_file, "features.txt", "File to write features to");
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
DEFINE_bool(debug, false, "Enable Debug Printing");

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


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  if (FLAGS_min_accuracy < 0.0)
    FLAGS_min_accuracy = 0.0;
  else if (FLAGS_min_accuracy > 1.0)
    FLAGS_min_accuracy = 1.0;

  // Read in the Examples
  // Also obtains variables that can be used (roots for synthesis)
  // and the possible input->output state pairs.
  unordered_set<Var> variables;
  vector<std::pair<string,string>> transitions;
  vector<Example> examples = ReadExamples(FLAGS_ex_file,
      variables,
      &transitions);

  std::reverse(transitions.begin(), transitions.end());

  examples = WindowExamples(examples, FLAGS_window_size);

  // Turning variables into roots
  vector<ast_ptr> inputs, roots;
  for (const Var& variable : variables) {
    if (variable.name_ != "goal" && variable.name_ != "free_path" && variable.name_ != "DoorState") {
      roots.push_back(make_shared<Var>(variable));
    }
  }

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

  // Loading Library Function Definitions
  vector<FunctionEntry> library = ReadLibrary(FLAGS_lib_file);

  cout << "----Library----" << endl;
  for (auto& func : library) {
    cout << func << endl;
  }
  cout << endl;

  // Enumerate features up to a fixed depth
  vector<Signature> signatures;
  vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library,
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
  ldipsL3(examples, transitions, ops, FLAGS_sketch_depth, FLAGS_min_accuracy,
      FLAGS_out_dir);
}
