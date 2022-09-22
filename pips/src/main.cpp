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

DEFINE_string(sketch_file, "", "Sketch file");
DEFINE_string(exs_file, "", "Examples file");
DEFINE_string(ops_file, "", "Operation library file");
DEFINE_uint32(max_depth, 3, "Maximum enumeration depth");
DEFINE_double(min_accuracy, 1.0,
              "What proportion of examples should be SAT to declare victory?");
DEFINE_bool(write_features, false, "Write all enumerated features to a file");
DEFINE_string(feature_file, "features.txt", "File to write features to");
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");

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

  if (FLAGS_sketch_file == "") {
    cout << "No sketch provided" << endl;
    return EXIT_FAILURE;
  }

  if (FLAGS_exs_file == "") {
    cout << "No examples file provided!" << endl;
    return EXIT_FAILURE;
  }

  if (FLAGS_ops_file == "") {
    cout << "No ops file provided!" << endl;
    return EXIT_FAILURE;
  }

  unordered_set<Var> variables;
  vector<Example> examples = ReadExamples(FLAGS_exs_file, variables);

  // Adding Library Function Definitions
  vector<FunctionEntry> library = ReadLibrary(FLAGS_ops_file);

  vector<ast_ptr> inputs, roots;
  for (const Var& variable : variables) {
    roots.push_back(make_shared<Var>(variable));
  }

  vector<Signature> signatures;
  vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library,
                                          FLAGS_max_depth, &signatures);

  void* sketch_handle = dlopen(FLAGS_sketch_file.c_str(), RTLD_LAZY);
  if (!sketch_handle) {
    cout << "Could not load sketch: " << dlerror() << endl;
    return EXIT_FAILURE;
  }

  typedef Sketch* (*get_sketch_t)();
  dlerror();
  get_sketch_t get_sketch = (get_sketch_t)dlsym(sketch_handle, "get_sketch");
  const char* error = dlerror();
  if (error) {
    cout << "Could not load sketch: " << error << endl;
    dlclose(sketch_handle);
    return EXIT_FAILURE;
  }

  // Copy the sketch onto the stack then delete and clean up.
  const Sketch* sketch_ptr = get_sketch();
  const Sketch sketch = *sketch_ptr;
  delete sketch_ptr;
  dlclose(sketch_handle);

  cout << "----Roots----" << endl;
  for (auto& node : roots) {
    cout << node << endl;
  }
  cout << endl;

  cout << "----Library----" << endl;
  for (auto& func : library) {
    cout << func << endl;
  }
  cout << endl;

  if (FLAGS_write_features) {
    ofstream feature_file;
    feature_file.open(FLAGS_feature_file);
    for (auto& op : ops) {
      feature_file << op << '\n';
    }
    feature_file.close();
  }

  cout << "---- Number Enumerated ----" << endl;
  cout << ops.size() << endl << endl;

  cout << "---- Sketch ----" << endl;
  for (const auto& p : sketch) {
    cout << p.first << " --> " << p.second << endl;
  }
  cout << endl;

  cout << "---- Solving ----" << endl;
  vector<ast_ptr> program =
      SolveConditional(examples, ops, sketch, FLAGS_min_accuracy);
  cout << endl;

  cout << "---- Program conditions ----" << endl;
  for (const ast_ptr& cond : program) {
    cout << cond << endl;
  }
  cout << endl;
}
