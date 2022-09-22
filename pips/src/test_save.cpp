#include <dlfcn.h>
#include <gflags/gflags.h>
#include <z3++.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "ast.hpp"
#include "enumeration.hpp"
#include "library_functions.hpp"
#include "parsing.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"
// #include "sketches/sketches.hpp"

DEFINE_string(ex_file, "examples/simple_atk.json", "Examples file");
DEFINE_string(lib_file, "ops/simple_atk.json", "Operation library file");
DEFINE_uint32(feat_depth, 3, "Maximum enumeration depth for features.");
DEFINE_uint32(sketch_depth, 3, "Maximum enumeration depth for sketch.");
DEFINE_uint32(window_size, 5, "Size of sliding window to subsample demonstrations with.");
DEFINE_double(min_accuracy, 1.0,
              "What proportion of examples should be SAT to declare victory?");
DEFINE_bool(write_features, false, "Write all enumerated features to a file");
DEFINE_string(feature_file, "features.txt", "File to write features to");
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
DEFINE_bool(debug, false, "Enable Debug Printing");

using namespace AST;
using namespace std;
using Eigen::Vector2f;
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
  unordered_set<std::pair<string,string>, pair_hash> transitions;
  vector<Example> examples = ReadExamples(FLAGS_ex_file,
      variables,
      &transitions);

  examples = WindowExamples(examples, FLAGS_window_size);

  // Turning variables into roots
  vector<ast_ptr> inputs, roots;
  for (const Var& variable : variables) {
    roots.push_back(make_shared<Var>(variable));
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

  // Enumerate possible sketches
  const auto sketches = AST::EnumerateSketches(FLAGS_sketch_depth);
  // For each input/output pair
  for (const auto& transition : transitions) {
    cout << transition.first << "<-" << transition.second << endl;
    for (const auto& sketch : sketches) {
      cout << sketch << endl;
      bool solved = false;
      ast_ptr solution =
          SolvePredicate(examples,
                  ops, sketch, transition, FLAGS_min_accuracy, &solved);
      if (solved) {
          // cout << "Solution: " << solution << endl;
          // Test writing ast as json to file
          // ofstream output_file;
          // output_file.open("synthd/mpdm_1/GoAlone_GoAlone.json");
          // const json output = solution->ToJson();
          // output_file << std::setw(4) << output << std::endl;
          // output_file.close();

          // Test reading it back in again
          ifstream input_file;
          input_file.open("synthd/mpdm_1/GoAlone_GoAlone.json");
          json loaded;
          input_file >> loaded;
          ast_ptr recovered = AST::AstFromJson(loaded);
          cout << "Recovered: " << recovered << endl;
      }
    }
    cout << endl;
  }
}
