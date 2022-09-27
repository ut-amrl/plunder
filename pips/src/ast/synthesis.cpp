#include "synthesis.hpp"

#include <gflags/gflags.h>
#include "Python.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <cfloat>

#include "../submodules/amrl_shared_lib/util/timer.h"
#include "enumeration.hpp"
#include "gflags/gflags_declare.h"
#include "parsing.hpp"
#include "utils/nd_bool_array.hpp"
#include "visitors/deepcopy_visitor.hpp"
#include "visitors/fillhole_visitor.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"
#include "visitors/tosmtlib_visitor.hpp"

using namespace std;
using AST::CheckModelAccuracy;
using nlohmann::json;

bool debug = false;
uint32_t batch_size;

namespace AST {

    PyObject *pName, *pModule, *pFunc;

    // Basically a breadth-first search of all combination of indices of the ops
    // vector starting at {0, 0, ..., 0}
    class index_iterator {
    public:
        index_iterator(size_t op_count, size_t holes_to_fill)
            : op_count_(op_count),
            holes_to_fill_(holes_to_fill),
            visited_(nd_bool_array(
                vector<size_t>(holes_to_fill_, op_count_))) {  // :^)
            // Some checks to make sure what we're doing is sensible.
            if (op_count_ == 0) {
                throw std::invalid_argument("no ops!");
            }
            if (holes_to_fill_ == 0) {
                throw std::invalid_argument("no holes to fill!");
            }

            // make the initial list of however many 0s
            vector<size_t> initial_coords;
            for (size_t i = 0; i < holes_to_fill_; ++i) {
                initial_coords.push_back(0);
            }

            // Add it to the queue to start BFS
            indicies_.push(initial_coords);
        }

        vector<size_t> next() {
            // Get the current lowest priority index list
            vector<size_t> current = indicies_.front();
            indicies_.pop();

            // Add neighbors of this index list to the queue
            for (size_t i = 0; i < holes_to_fill_; ++i) {
                // if we've reached an edge, don't do anything
                if (current[i] == op_count_ - 1) {
                    continue;
                }

                // Copy current index list, increase one index by 1, add it to the
                // queue, and mark it as visited so it's not added again.
                vector<size_t> copy = current;
                copy[i] += 1;
                if (!visited_.get(copy)) {
                    indicies_.push(copy);
                    visited_.set(copy, true);
                }
            }
            return current;
        }

        bool has_next() const { return !indicies_.empty(); }

        vector<size_t> zeros() const { return vector<size_t>(holes_to_fill_, 0); }

    private:
        const size_t op_count_;
        const size_t holes_to_fill_;
        std::queue<vector<size_t>> indicies_;
        nd_bool_array visited_;
    };

    // Helper Function for scoring a candidate predicate given only a set
    // of examples and the desired transition
    float ScorePredicate(ast_ptr pred, const pair<string, string> &transition,
                        const vector<Example> &examples, float *pos, float *neg) {
        // Checking Initial accuracy
        const SymEntry out(transition.second);
        const SymEntry in(transition.first);

        // Split up all the examples into a "yes" set or "no" sets
        unordered_set<Example> yes;
        unordered_set<Example> no;
        SplitExamples(examples, transition, &yes, &no);
        return CheckModelAccuracy(pred, yes, no, pos, neg);
    }

    // Utility function that converts a Z3 model (solutions for parameter holes)
    // into a C++ unordered_map associating the names of parameter holes with
    // floats.
    Model Z3ModelToMap(const z3::model &z3_model) {
        Model ret;
        for (size_t i = 0; i < z3_model.size(); ++i) {
            const z3::func_decl v = z3_model[i];
            const z3::expr interp = z3_model.get_const_interp(v);
            if (!interp.is_numeral()) {
                throw std::invalid_argument("All model values must be numeral");
            }
            // Keep these as int64_t, otherwise you might get overflow errors and Z3
            // will complain.
            const int64_t numerator = interp.numerator().get_numeral_int64();
            const int64_t denominator = interp.denominator().get_numeral_int64();
            const float value = (float)numerator / denominator;
            const string key = v.name().str();
            Num value_ast(value, {0, 0, 0});
            ret[key] = make_shared<Num>(value_ast);
        }
        return ret;
    }

    CumulativeFunctionTimer solve_smtlib("SolveSMTLIBProblem");
    Model SolveSMTLIBProblem(const string &problem) {
        CumulativeFunctionTimer::Invocation invoke(&solve_smtlib);
        z3::context context;
        z3::optimize solver(context);
        solver.from_string(problem.c_str());
        z3::check_result result = solver.check();
        if (result == z3::sat) {
            z3::model m = solver.get_model();
            return Z3ModelToMap(m);
        } else {
            throw std::invalid_argument("UNSAT");
        }
    }

    CumulativeFunctionTimer make_smtlib("MakeSMTLIBProblem");
    string MakeSMTLIBProblem(const unordered_set<Example> &yes,
                            const unordered_set<Example> &no,
                            const ast_ptr program, const bool srtr) {
        CumulativeFunctionTimer::Invocation invoke(&make_smtlib);

        // Create empty sets to hold data needed for program generation. The params
        // set will track the names of parameter holes discovered, and the
        // assertions set will hold the assertions we generate.
        unordered_set<string> params;
        unordered_set<string> assertions;

        // Make a new set that includes all examples.
        unordered_set<Example> all_examples(yes);
        all_examples.insert(no.cbegin(), no.cend());
        // For every example, ...

        for (const Example &example : all_examples) {
            // Partially evaluate the program with the examples as much as possible.
            ast_ptr partial = Interpret(program, example);
            // Create an SMT-LIB expression from the program and that example.

            ToSMTLIB converter = AstToSMTLIB(partial, example);
            const string smtlib = converter.Get();

            // Add full assertions using that expression to our assertion list. It's
            // important to check whether the example is in both sets because we
            // could have contradictory examples.
            if (yes.find(example) != yes.cend())
                assertions.insert("(assert-soft " + smtlib + ")");
            if (no.find(example) != no.cend())
                assertions.insert("(assert-soft (not " + smtlib + "))");

            // If there are any parameter holes, make note of them too.
            const unordered_set<string> example_params = converter.GetParams();
            params.insert(example_params.cbegin(), example_params.cend());
        }

        // Create a string where our full SMT-LIB problem will be created.
        //
        // Any SMT-LIB formula including division by 0 is SAT, so create a new
        // function for "safe" division.
        //
        // TODO(simon) Find a smarter solution, since this could still possibly
        // result in SATs when it shouldn't.
        string problem = R"(
        (define-fun safe-div ((x!1 Real) (x!2 Real)) Real
        (ite (= x!2 0.0)
            0.0
            (/ x!1 x!2)))

        (define-fun cross ((x!1 Real) (y!1 Real) (x!2 Real) (y!2 Real)) Real
        (+ (* x!1 y!2) (* y!1 x!2)))

        (define-fun dot ((x!1 Real) (y!1 Real) (x!2 Real) (y!2 Real)) Real
        (+ (* x!1 x!2) (* y!1 y!2)))

        (define-fun sq ((x!1 Real)) Real
        (* x!1 x!1))

        (define-fun euc-dist-sq ((x!1 Real) (y!1 Real) (x!2 Real) (y!2 Real)) Real
        (+ (sq (- x!2 x!1)) (sq (- y!2 y!1))))

        (define-fun norm-sq ((x!1 Real) (y!1 Real)) Real
        (+ (sq x!1) (sq y!1)))

        (define-fun vec-x ((x!1 Real) (y!1 Real)) Real
        x!1)

        (define-fun vec-y ((x!1 Real) (y!1 Real)) Real
        y!1)
    )";

        problem = "";

        // For every parameter hole discovered, add a real constant to the problem.
        for (const string &param : params) {
            problem += "(declare-const " + param + " Real)\n";
        }

        // Add every assertion created previously to the problem.
        for (const string &assertion : assertions) {
            problem += assertion + "\n";
        }

        if (srtr) {
            for (const string &param : params) {
                // minimize the absolute value of these constants. Closer to 0 is
                // better in the SRTR case, and ignoreable the rest of the time
                // (because of lexicographic optimization)
                const string absolute =
                    "(ite (> 0 " + param + ") (- 0 " + param + ") " + param + ")";
                problem += "(minimize " + absolute + ")\n";
            }
        }

        // Now that the problem string has been completely generated, return it.
        // There is no need to add instructions to the solver like (check-sat) or
        // (get-model) since those actions will be performed by us using the Z3 C++
        // API later.
        return problem;
    }

    // Takes a sketch and fills in holes in logistic expressions with the constants provided
    ast_ptr FillLogHoles(ast_ptr sketch, vector<float> a_vals, vector<float> x_0_vals) {
        // Iterate through conjunctions/disjunctions
        ast_ptr *pointer = &sketch;
        int index = 0;
        while (pointer != nullptr) {
            bin_ptr op = dynamic_pointer_cast<BinOp>(*pointer);
            bool stop_flag = false;

            if (op == nullptr || (op->op_ != "And" && op->op_ != "Or")) {
                stop_flag = true;
            }

            // Evaluate all examples on the sketch
            feature_ptr feature;
            if(stop_flag)
                feature = dynamic_pointer_cast<Feature>(*pointer);
            else
                feature = dynamic_pointer_cast<Feature>(op->left_);
            bin_ptr flip = dynamic_pointer_cast<BinOp>(feature->current_value_);
            tern_ptr logistic = dynamic_pointer_cast<TernOp>(flip->left_);
            logistic->a_ = make_shared<Num>(Num(a_vals[index], {0, 0, 0}));
            logistic->b_ = make_shared<Num>(Num(x_0_vals[index], {0, 0, 0}));
            index++;

            if(stop_flag)
                break;
            
            // Next clause
            pointer = &(op->right_);
        }
        return sketch;
    }

    // Attempts to solve for a satisfactory assignment of real values to
    // parameter holes that satisfies a subset of examples. Returns the
    // percentage of examples satisfied.
    double PredicateL1(ast_ptr sketch, const unordered_set<Example> &pos,
                    const unordered_set<Example> &neg, const bool srtr) {
        const string problem = MakeSMTLIBProblem(pos, neg, sketch, srtr);
        try {
            const Model solution = SolveSMTLIBProblem(problem);
            if (solution.empty()) {
                throw std::invalid_argument("UNSAT");
            }
            FillHoles(sketch, solution);
            const double output = CheckModelAccuracy(sketch, pos, neg);
            return output;
        } catch (const std::invalid_argument) {
            return 0.0;
        }
    }

    // Attempts to solve for the most likely assignment of real values for the
    // logistic equation. Returns the log likelihood and modifies sketch
    vector<double> LikelihoodPredicateL1(vector<ast_ptr> sketches, const unordered_set<Example> &pos,
                                const unordered_set<Example> &neg,
                                const bool srtr,
                                uint32_t& sketches_completed) {

        // Generate y_j (tells us whether an example satisfied a transition)
        vector<bool> y_j(neg.size() + pos.size(), false);
        fill_n(y_j.begin(), pos.size(), true);

        vector<vector<char>> clauses_arr;
        vector<vector<vector<float>>> expressions_arr;
        for(ast_ptr sketch : sketches){
            // Iterate through conjunctions/disjunctions
            vector<char> clauses;
            ast_ptr *pointer = &sketch;
            vector<vector<float>> expressions;
            while (pointer != nullptr) {
                bin_ptr op = dynamic_pointer_cast<BinOp>(*pointer);
                bool stop_flag = false;

                if (op == nullptr) stop_flag = true;
                else {
                    // Generate clauses array
                    if (op->op_ == "And") {
                        clauses.push_back('&');
                    } else if (op->op_ == "Or") {
                        clauses.push_back('|');
                    } else {
                        stop_flag = true;
                    }
                }
                
                // Evaluate all examples on the sketch
                vector<float> E_k;
                feature_ptr feature;
                if(stop_flag)
                    feature = dynamic_pointer_cast<Feature>(*pointer);
                else
                    feature = dynamic_pointer_cast<Feature>(op->left_);
                bin_ptr flip = dynamic_pointer_cast<BinOp>(feature->current_value_);
                tern_ptr logistic = dynamic_pointer_cast<TernOp>(flip->left_);
                ast_ptr E = logistic->x_;
                for (Example each : pos) {
                    num_ptr res = dynamic_pointer_cast<Num>(Interpret(E, each));
                    E_k.push_back(res->value_);
                }
                for (Example each : neg) {
                    num_ptr res = dynamic_pointer_cast<Num>(Interpret(E, each));
                    E_k.push_back(res->value_);
                }
                expressions.push_back(E_k);

                if(stop_flag)
                    break;
                
                // Next clause
                pointer = &(op->right_);
            }
            clauses_arr.push_back(clauses);
            expressions_arr.push_back(expressions);
        }

        // Call optimizer here
        vector<vector<float>> a_vals_arr;
        vector<vector<float>> x_0_vals_arr;
        vector<double> sol_arr;

        PyObject *pClauses_arr, *pY_j, *pE_k_arr, *pArgs, *pValue;
        

        // Arguments
        pClauses_arr = PyList_New(sketches.size());
        for(int o=0; o<sketches.size(); o++){
            vector<char> clauses = clauses_arr[o];
            PyObject *pClauses = PyList_New(clauses.size());
            for(int i = 0; i < clauses.size(); i++){
                PyList_SetItem(pClauses, i, clauses[i] == '&' ? PyLong_FromLong(0) : PyLong_FromLong(1));
            }
            PyList_SetItem(pClauses_arr, o, pClauses); 
        }

        pY_j = PyList_New(y_j.size());
        for(int i = 0; i < y_j.size(); i++){
            PyList_SetItem(pY_j, i, (y_j[i] ? PyLong_FromLong(1) : PyLong_FromLong(0)));
        }
        pE_k_arr = PyList_New(sketches.size());
        for(int o=0; o<sketches.size(); o++){
            vector<vector<float>> expressions = expressions_arr[o];
            PyObject *pE_k = PyList_New(expressions.size());
            for(int i = 0; i < expressions.size(); i++){
                PyObject *subarr;
                subarr = PyList_New(expressions[i].size());
                for(int j = 0; j < expressions[i].size(); j++){
                    PyList_SetItem(subarr, j, PyFloat_FromDouble(expressions[i][j]));
                }
                PyList_SetItem(pE_k, i,  subarr);
            }
            PyList_SetItem(pE_k_arr, o, pE_k);
        }

        pArgs = PyTuple_Pack(3, pE_k_arr, pY_j, pClauses_arr);

        if (!pArgs) {
            fprintf(stderr, "Cannot convert argument\n");
        }

        // Call function
        pValue = PyObject_CallObject(pFunc, pArgs);

        if (pValue != NULL && PyList_Check(pValue)) {
            // Retrieve results
            for(int i=0; i<sketches.size(); i++){
                PyObject* curExampleTuple = PyList_GetItem(pValue, i);
                double sol = PyFloat_AsDouble(PyTuple_GetItem(curExampleTuple, 0));
                PyObject* curExampleCoefs = PyTuple_GetItem(curExampleTuple, 1);
                vector<float> a_vals;
                vector<float> x_0_vals;
                PyList_Check(curExampleCoefs);
                for(int j = 0; j < PyList_Size(curExampleCoefs) / 2; j++){
                    a_vals.push_back(PyFloat_AsDouble(PyList_GetItem(curExampleCoefs, j)));
                }
                for(int j = PyList_Size(curExampleCoefs) / 2; j < PyList_Size(curExampleCoefs); j++){
                    x_0_vals.push_back(PyFloat_AsDouble(PyList_GetItem(curExampleCoefs, j)));
                }
                a_vals_arr.push_back(a_vals);
                x_0_vals_arr.push_back(x_0_vals);
                sol_arr.push_back(sol);
                sketches[i] = FillLogHoles(sketches[i], a_vals, x_0_vals);

                // Debug solutions
                if(debug){
                    cout << "Sketch: " << sketches[i] << " has score " << sol_arr[i] << endl;
                }
            }

        } else {
            PyErr_Print();
            fprintf(stderr,"Call failed\n");
        }

        sketches_completed += sketches.size();
        cout << "Transition total sketches completed: " << sketches_completed << endl;
        return sol_arr;
    }

    ast_ptr FillFeatureHoles(ast_ptr sketch, const vector<size_t> &indicies,
                            const vector<ast_ptr> &ops) {
        Model m;

        // Get a list of the names of all the feature holes in the conditional,
        // then store them in a vector because being able to access them in a
        // consistent order and by index is important for something we do later.
        const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
            MapFeatureHoles(sketch);

        vector<string> feature_holes;
        for (const auto &p : feature_hole_map) {
            feature_holes.push_back(p.first);
        }
        const size_t feature_hole_count = feature_holes.size();

        // For every feature hole...
        for (size_t i = 0; i < feature_hole_count; ++i) {
            // Get the name of a feature hole to fill and a possible value for it.
            const string &feature_hole = feature_holes[i];
            const pair<Type, Dimension> feature_hole_info =
                feature_hole_map.at(feature_hole);
            const Type feature_hole_type = feature_hole_info.first;
            const Dimension feature_hole_dims = feature_hole_info.second;
            const size_t index = indicies[i];
            const ast_ptr &op = ops[index];
            m[feature_hole] = op;
        }

        // If after creating the model the number of filled holes is not the same
        // as the number of holes that ought to be filled (indicating a type
        // error), we can give up and try the next model.
        if (m.size() != feature_hole_count) {
            return nullptr;
        }

        // Since no errors occured while creating the model, we can take the hole
        // values from it and use them to fill the holes in a copy of the
        // condition.
        ast_ptr filled = DeepCopyAST(sketch);
        filled->priority = FillHoles(filled, m);
        return filled;
    }

    CumulativeFunctionTimer pred_l2("PredicateL2");
    ast_ptr PredicateL2(const vector<Example> &examples, const vector<ast_ptr> &ops,
                        ast_ptr sketch, const pair<string, string> &transition,
                        const double min_accuracy, float *solved) {
        CumulativeFunctionTimer::Invocation invoke(&pred_l2);

        const SymEntry out(transition.second);
        const SymEntry in(transition.first);

        // Get a list of the names of all the feature holes in the conditional,
        // then store them in a vector because being able to access them in a
        // consistent order and by index is important for something we do later.
        const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
            MapFeatureHoles(sketch);
        vector<string> feature_holes;
        for (const auto &p : feature_hole_map) {
            feature_holes.push_back(p.first);
        }
        const size_t feature_hole_count = feature_holes.size();

        // Split up all the examples into a "yes" set or a "no" set based on
        // whether the result for the example matches the current example's
        // behavior.
        unordered_set<Example> yes;
        unordered_set<Example> no;
        SplitExamples(examples, transition, &yes, &no);

        if (debug) {
            cout << "Current Sketch: " << sketch << endl;
        }

        // Start iterating through possible models. index_iterator is explained
        // seperately.
        ast_ptr solution_cond = sketch;
        float current_best = 0.0;
        if (feature_hole_count > 0) {
            index_iterator c(ops.size(), feature_hole_count);
            solution_cond = nullptr;
            bool keep_searching = true;
            int count = 0.0;

    #pragma omp parallel
            while (keep_searching) {
                // Use the indices given to us by the iterator to select ops for
                // filling our feature holes and create a model.
                vector<size_t> op_indicies;
    #pragma omp critical
                {
                    if (c.has_next()) {
                        op_indicies = c.next();
                    } else {
                        keep_searching = false;
                        op_indicies = c.zeros();
                    }
                    count++;
                }

                ast_ptr filled = FillFeatureHoles(sketch, op_indicies, ops);
                if (filled != nullptr) {
                    const double sat_ratio = PredicateL1(filled, yes, no, false);
    #pragma omp critical
                    {
                        if (keep_searching && sat_ratio >= min_accuracy) {
                            keep_searching = false;
                            solution_cond = filled;
                            current_best = sat_ratio;
                        } else if (sat_ratio > current_best ||
                                sat_ratio == current_best &&
                                    filled->priority < solution_cond->priority) {
                            solution_cond = filled;
                            current_best = sat_ratio;
                        }
                    }
                }
            }
        } else {
            // Since no errors occured while creating the model, we can take the
            // hole values from it and use them to fill the holes in a copy of the
            // condition.
            ast_ptr cond_copy = DeepCopyAST(sketch);
            const double sat_ratio = PredicateL1(cond_copy, yes, no, false);
    #pragma omp critical
            {
                if (sat_ratio >= current_best) {
                    solution_cond = cond_copy;
                    current_best = sat_ratio;
                }
            }
        }

        // Return the solution if one exists, otherwise return a nullptr.
        if (solution_cond != nullptr) {
            *solved = current_best;
        }
        return solution_cond;
    }

    CumulativeFunctionTimer likelihood_pred_l2("LikelihoodPredicateL2");
    ast_ptr LikelihoodPredicateL2(const vector<Example> &examples,
                                const vector<ast_ptr> &ops, ast_ptr sketch,
                                const pair<string, string> &transition,
                                const double max_error, float *best_error) {
        CumulativeFunctionTimer::Invocation invoke(&likelihood_pred_l2);
        const SymEntry out(transition.second);
        const SymEntry in(transition.first);

        uint32_t sketches_completed=0;

        // Get a list of the names of all the feature holes in the conditional,
        // then store them in a vector because being able to access them in a
        // consistent order and by index is important for something we do later.
        const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
            MapFeatureHoles(sketch);
        vector<string> feature_holes;
        for (const auto &p : feature_hole_map) {
            feature_holes.push_back(p.first);
        }
        const size_t feature_hole_count = feature_holes.size();

        // Split up all the examples into a "yes" set or a "no" set based on
        // whether the result for the example matches the current example's
        // behavior.
        unordered_set<Example> yes;
        unordered_set<Example> no;
        SplitExamples(examples, transition, &yes, &no);

        if (debug) {
            cout << "Current Sketch: " << sketch << endl;
        }

        // Start iterating through possible models. index_iterator is explained
        // seperately.
        ast_ptr solution_cond = sketch;
        float current_best = -1;
        if (feature_hole_count > 0) {
            index_iterator c(ops.size(), feature_hole_count);
            solution_cond = nullptr;
            bool keep_searching = true;
            int count = 0.0;


            while (keep_searching) {
                // Use the indices given to us by the iterator to select ops for
                // filling our feature holes and create a model.
                vector<ast_ptr> arr_filled;
                for(int i=0; i<batch_size; i++) {
                    vector<size_t> op_indicies;
                    if (c.has_next()) {
                        op_indicies = c.next();
                        ast_ptr filled = FillFeatureHoles(sketch, op_indicies, ops);
                        arr_filled.push_back(filled);
                    } else {
                        keep_searching = false;
                        break;
                    }
                }

                vector<double> log_likelihoods = LikelihoodPredicateL1(arr_filled, yes, no, false, sketches_completed);

                // best within the last NUM_CORES sketches
                double best_log_likelihood = DBL_MAX;
                ast_ptr best_filled;

                for(int i=0; i<log_likelihoods.size(); i++){
                    double d = log_likelihoods[i];
                    if(d < best_log_likelihood) {
                        best_log_likelihood = d;
                        best_filled = arr_filled[i];
                    }
                }

                if(best_filled != nullptr) {
                    if (keep_searching && best_log_likelihood <= max_error) {
                        keep_searching = false;
                        solution_cond = best_filled;
                        current_best = best_log_likelihood;
                    } else if (current_best == -1 ||
                            best_log_likelihood < current_best ||
                            (best_log_likelihood == current_best &&
                                best_filled->priority < solution_cond->priority)) {
                        solution_cond = best_filled;
                        current_best = best_log_likelihood;
                    }
                }
            }
        } else {
            // Since no errors occured while creating the model, we can take the
            // hole values from it and use them to fill the holes in a copy of the
            // condition.
            ast_ptr cond_copy = DeepCopyAST(sketch);
            const double log_likelihood = PredicateL1(cond_copy, yes, no, false);
            if (log_likelihood <= current_best) {
                solution_cond = cond_copy;
                current_best = log_likelihood;
            }
        }
        // Return the solution if one exists, otherwise return a nullptr.
        if (solution_cond != nullptr) {
            *best_error = current_best;
        }
        return solution_cond;
    }

    ast_ptr ldipsL2(ast_ptr candidate, const vector<Example> &examples,
                    const vector<ast_ptr> &ops,
                    const pair<string, string> &transition,
                    const float min_accuracy, ast_ptr best_program,
                    float *best_score) {
        // Find best performing completion of current sketch
        float solved = 0.0;
        ast_ptr solution = PredicateL2(examples, ops, candidate, transition,
                                    min_accuracy, &solved);

        // Keep if beats best performing solution.
        if (solved > *best_score) {
            *best_score = solved;
            best_program = solution;
        }
        Z3_reset_memory();
        return best_program;
    }

    pair<ast_ptr, float> emdipsL2(ast_ptr candidate,
                                const vector<Example> &examples,
                                const vector<ast_ptr> &ops,
                                const pair<string, string> &transition,
                                const float max_error) {
        // Find best performing completion of current sketch
        float error = -1;
        ast_ptr solution = LikelihoodPredicateL2(examples, ops, candidate,
                                                transition, max_error, &error);

        return pair<ast_ptr, float>(solution, error);
    }

    vector<ast_ptr> ldipsL3(const vector<Example> &demos,
                            const vector<pair<string, string>> &transitions,
                            const vector<ast_ptr> lib, const int sketch_depth,
                            const float min_accuracy, const string &output_path) {
        vector<Example> examples = demos;
        // Enumerate possible sketches
        const auto sketches = EnumerateSketches(sketch_depth);
        cout << "Number of sketches: " << sketches.size() << endl;

        vector<ast_ptr> transition_solutions;

        // For each input/output pair
        for (const auto &transition : transitions) {
            // Skipping already synthesized conditions, allows for very basic
            // checkpointing.
            const string output_name =
                output_path + transition.first + "_" + transition.second + ".json";
            if (ExistsFile(output_name)) {
                continue;
            }
            // if (transition.first != "GoAlone" || transition.second != "Pass") {
            // continue;
            // }
            cout << "----- " << transition.first << "->";
            cout << transition.second << " -----" << endl;
            float current_best = 0.0;
            ast_ptr current_solution = nullptr;
            for (const auto &sketch : sketches) {
                // Attempt L2 Synthesis with current sketch.
                current_solution =
                    ldipsL2(sketch, examples, lib, transition, min_accuracy,
                            current_solution, &current_best);
                if (current_best >= min_accuracy) break;
                if (debug) {
                    cout << "Score: " << current_best << endl;
                    cout << "Solution: " << current_solution << endl;
                    cout << "- - - - -" << endl;
                }
            }
            // Write the solution out to a file.
            cout << "Score: " << current_best << endl;
            cout << "Final Solution: " << current_solution << endl;
            ofstream output_file;
            output_file.open(output_name);
            const json output = current_solution->ToJson();
            output_file << std::setw(4) << output << std::endl;
            output_file.close();

            // Filter out Examples used by this transition
            examples = FilterExamples(examples, transition);

            cout << endl;
            transition_solutions.push_back(current_solution);
        }
        return transition_solutions;
    }

    EmdipsOutput emdipsL3(const vector<Example> &demos,
                        const vector<pair<string, string>> &transitions,
                        const vector<ast_ptr> lib, const int sketch_depth,
                        const vector<float> max_error,
                        const string &output_path,
                        const uint32_t b_size) {

        batch_size = b_size;
        
        // Initialize python support
        Py_Initialize();
        PyRun_SimpleString("import sys, os; sys.path.insert(0, os.path.join('./', 'pips/', 'src/', 'optimizer'))");

        // File name
        pName = PyUnicode_FromString((char*)"optimizer");

        pModule = PyImport_Import(pName);
        Py_DECREF(pName);

        if(pModule != NULL) {
            // Function name
            pFunc = PyObject_GetAttrString(pModule, (char*)"run_optimizer_threads");

            if(!(pFunc && PyCallable_Check(pFunc))) {
                if (PyErr_Occurred())
                    PyErr_Print();
                fprintf(stderr, "Cannot find optimization function\n");
            }
        } else {
            PyErr_Print();
            fprintf(stderr, "Failed to load optimization file");
        }

        vector<Example> examples = demos;
        // Enumerate possible sketches
        const auto sketches = EnumerateSketches(sketch_depth);
        cout << "Number of sketches: " << sketches.size() << endl;
        for (ast_ptr each : sketches) {
            cout << each << endl;
        }
        cout << endl << endl;

        vector<ast_ptr> transition_solutions;

        vector<float> log_likelihoods;

        // For each input/output pair
        for (int t = 0; t < transitions.size(); t++) {
            const auto &transition = transitions[t];
            // Skipping already synthesized conditions, allows for very basic
            // checkpointing.
            const string output_name =
                output_path + transition.first + "_" + transition.second + ".json";
            if (ExistsFile(output_name)) {
                continue;
            }

            cout << endl << endl << "----- " << transition.first << "->";
            cout << transition.second << " -----" << endl;
            cout << "Target log likelihood: < " << max_error[t] << endl;

            unordered_set<Example> yes;
            unordered_set<Example> no;
            SplitExamples(examples, transition, &yes, &no);
            cout << "Num transitions (pos): " << yes.size() << endl;
            cout << "Num transitions (neg): " << no.size() << endl;

            float current_best = -1;
            ast_ptr current_solution = nullptr;
            if (yes.size() == 0) {
                current_best = 0.0;
                current_solution = make_shared<Bool>(Bool(false));
            } else if (no.size() == 0) {
                current_best = 0.0;
                current_solution = make_shared<Bool>(Bool(true));
            } else {
                for (const auto &sketch : sketches) {
                    std::chrono::steady_clock::time_point timerBegin =
                        std::chrono::steady_clock::now();

                    // Attempt L2 Synthesis with current sketch.
                    pair<ast_ptr, float> new_solution =
                        emdipsL2(sketch, examples, lib, transition, max_error[t]);

                    if (current_best == -1 || new_solution.second < current_best) {
                        current_best = new_solution.second;
                        current_solution = new_solution.first;
                    }

                    if (debug) {
                        cout << "Error (Log likelihood): " << new_solution.second
                            << endl;
                        cout << "Solution: " << new_solution.first << endl;
                        std::chrono::steady_clock::time_point timerEnd =
                            std::chrono::steady_clock::now();
                        cout << "Time Elapsed: "
                            << ((float)(std::chrono::duration_cast<
                                            std::chrono::milliseconds>(timerEnd -
                                                                        timerBegin)
                                            .count())) /
                                    1000.0
                            << endl;
                        cout << "- - - - -" << endl;
                    }
                    if (current_best < max_error[t] || current_best == 0.0) break;
                }
            }
            // Write the solution out to a file.
            cout << "Error (Log likelihood): " << current_best << endl;
            cout << "Final Solution: " << current_solution << endl;
            ofstream output_file;
            output_file.open(output_name);
            const json output = current_solution->ToJson();
            output_file << std::setw(4) << output << std::endl;
            output_file.close();

            // Filter out Examples used by this transition
            examples = FilterExamples(examples, transition);

            transition_solutions.push_back(current_solution);
            log_likelihoods.push_back(current_best);
        }
        
        EmdipsOutput res;
        res.ast_vec = transition_solutions;
        res.log_likelihoods = log_likelihoods;

        // Clean up python
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
        Py_Finalize();

        return res;
    }

    void DIPR(const vector<Example> &demos, const vector<ast_ptr> &programs,
            const vector<pair<string, string>> &transitions,
            const vector<ast_ptr> lib, const int sketch_depth,
            const float min_accuracy, const string &output_path) {
        vector<Example> examples = demos;
        // Enumerate possible sketches
        const auto sketches = EnumerateSketches(sketch_depth);

        // For each input/output pair
        for (size_t i = 0; i < programs.size(); ++i) {
            // TODO(jaholtz) make sure programs and transitions are in the same
            // order.
            const auto transition = transitions[i];
            ast_ptr best_program = programs[i];
            cout << "----- " << transition.first << "->";
            cout << transition.second << " -----" << endl;
            cout << "Initial Program:" << best_program << endl;

            // Checking Initial accuracy
            float pos = 0;
            float neg = 0;
            float best_score =
                ScorePredicate(best_program, transition, examples, &pos, &neg);

            cout << "Initial Score: " << best_score << endl;
            if (best_score >= min_accuracy) {
                cout << "Sufficient Performance, Skipping" << endl;
                cout << endl;
                ofstream output_file;
                const string output_name = output_path + transition.first + "_" +
                                        transition.second + ".json";
                output_file.open(output_name);
                const json output = best_program->ToJson();
                output_file << std::setw(4) << output << std::endl;
                output_file.close();
                continue;
            }

            // Search all possible extensions of the current program.
            // TODO(jaholtz) iterative over both sketches separately
            for (const auto &sketch : sketches) {
                // Extend the Sketch
                if (debug) {
                    cout << "Pos: " << pos << " Neg: " << neg << endl;
                }
                ast_ptr candidate =
                    ExtendPred(programs[i], sketch, sketch, pos, neg);

                // Attempt L2 Synthesis with the extended sketch.
                best_program = ldipsL2(candidate, examples, lib, transition,
                                    min_accuracy, best_program, &best_score);
                Z3_reset_memory();

                if (best_score >= min_accuracy) break;
            }
            // Write the solution out to a file.
            cout << "Final Solution: " << best_program << endl;
            cout << "Final Score: " << best_score << endl;
            ofstream output_file;
            const string output_name =
                output_path + transition.first + "_" + transition.second + ".json";
            output_file.open(output_name);
            const json output = best_program->ToJson();
            output_file << std::setw(4) << output << std::endl;
            output_file.close();

            // Filter out Examples used by this transition
            examples = FilterExamples(examples, transition);
            cout << endl;
        }
    }

    void SRTR(const vector<Example> &demos, const vector<ast_ptr> &programs,
            const vector<pair<string, string>> &transitions,
            const string &output_path) {
        // For each branch in the existing sketch
        for (size_t i = 0; i < programs.size(); ++i) {
            ast_ptr sketch = programs[i];
            const auto transition = transitions[i];

            // Checking Initial accuracy
            const SymEntry out(transition.second);
            const SymEntry in(transition.first);

            // Split up all the examples into a "yes" set or a "no" set based on
            // whether the result for the example matches the current example's
            // behavior.
            unordered_set<Example> yes;
            unordered_set<Example> no;
            SplitExamples(demos, transition, &yes, &no);
            if (yes.size() + no.size() > 0) {
                cout << "----- " << transition.first << "->";
                cout << transition.second << " -----" << endl;
                cout << "Initial Program:" << endl << sketch << endl;

                float current_best = CheckModelAccuracy(sketch, yes, no);
                cout << "Initial Score: " << current_best << endl;

                // Find best performing completion of current sketch
                sketch = Srtrize(sketch);
                const double score = PredicateL1(sketch, yes, no, true);

                cout << "Adjusted Program: " << endl << sketch << endl;
                cout << "Final Score: " << score << endl;
                cout << "- - - - - -" << endl;
                Z3_reset_memory();

                // Write the solution out to a file.
                ofstream output_file;
                const string output_name = output_path + transition.first + "_" +
                                        transition.second + ".json";
                output_file.open(output_name);
                const json output = sketch->ToJson();
                output_file << std::setw(4) << output << std::endl;
                output_file.close();

                // Filter out Examples used by this transition
                // examples = FilterExamples(examples, transition);
                cout << endl;
            }
        }
    }

}  // namespace AST