#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <assert.h>
#include <stdlib.h>
#include <filesystem>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <cmath>
#include <cassert>
#include <numeric>
#include <math.h>
#include <cstring>
#include <optional>
#include <chrono>

#include <dlfcn.h>
#include <z3++.h>

#include "ast/ast.hpp"
#include "ast/enumeration.hpp"
#include "ast/library_functions.hpp"
#include "ast/parsing.hpp"
#include "visitors/interp_visitor.hpp"
#include "ast/synthesis.hpp"

typedef int HA;

// -------------------- Settings --------------------
// These generally should not be touched, unless you fully understand their effects

// I/O Settings
const string GEN_ASP = "out/asp_iter";                      // ASPs generated by EMDIPS
const string TRAINING_TRAJ = "out/training_traj/iter";      // Trajectories (high-level action sequences) generated by particle filter
const string VALIDATION_TRAJ = "out/validation_traj/iter";  // Trajectories generated from ASPs, for all robots
const string PLOT_PATH = "plots/";                          // Plots
const string LOG_OBS_PATH = "out/log_obs";                  // Cumulative log observation likelihood across all runs
const string PCT_ACCURACY = "out/pct_acc";                  // Percent accuracy compared to the given trajectory
const string OPTIMIZER_PATH = "../pips/src/optimizer";      // Path to optimizer (for Python support)
const string OPERATION_LIB = "../pips/ops/emdips_operations.json"; // Path to operation library

// Precision Settings
const double EPSILON = 10E-10;
const int PRECISION = 10;

// Optimization Parameters
const int OPT_METHOD = 1;               // Optimization method:
                                        // 0: local (BFGS)
                                        // 1: local (L-BFGS-B)
                                        // 2: basin hopping
                                        // 3: dual annealing
                                        // 4: DIRECT
const bool ENUMERATE_SIGNS = false;     // Equivalent to enumerating over > and <
const bool PRINT_DEBUG = false;         // Extra debugging info
const int INITIAL_VALUES = 4;           // Initial values for x_0: 0 = all zeros, 1 = average, >1 = do all of the above, then enumerate over random initial guesses (use this to specify how many)
const double INIT_ALPHA = 0.0;          // Starting slope
const double OUTLIER_MAX = 20;          // Max negative log likelihood that an example can contribute to the total log likelihood
const int MAX_ITER = 300;               // Max number of iterations of a single optimization run
const int SAMPLE_SIZE = 100;            // Number of trajectories to process then pass into EMDIPS, per robot
const int EX_SAMPLED = 2000;            // Number of examples to be optimized over

// EM Loop parameters
const double POINT_ACCURACY = 0.9;      // randSwitch() parameter
const int STRUCT_CHANGE_FREQ = 3;       // Only enumerate over new program structures after this many iterations, else tune parameters for previous best structure

// Plot parameters
const int PLOT_TIME = 300;              // Maximum time step plotted

// EMDIPS parameters
const int PROG_ENUM = 1024;              // Number of programs to enumerate and optimize per iteration. For more complex program spaces, this may need to be higher
const int BASE_FEAT_DEPTH = 2;          // Feature (expression) depth in the first iteration. If your expressions are particularly complex, it may help to increase this. We suggest between 2 and 3

// Particle filter parameters
const int NUM_PARTICLES = 2000;         // Number of particle trajectories used in the particle filter
const double RESAMPLE_THRESHOLD = 10.0; // Resampling rate in the particle filter. Higher = more resampling. Default: resample at every timestep
double TEMPERATURE = 1.5;                 // Initial observation likelihood strength. Used to encourage exploration to avoid local minima. Default: off
const double TEMP_CHANGE = 0.05;           // TEMPERATURE decreases linearly by this much each iteration. Encourages exploration in earlier iterations, then convergence in later iterations.



// -------------------- Baselines --------------------
// This is used for testing against baselines.

enum LABELER {
    PERFECT,            // Perfect labels given
    GREEDY_HEURISTIC,   // Greedily label actions
    STABLE_HEURISTIC,   // Use randSwitch()
};

enum SYNTHESIS_ALGO {
    EMDIPS,             // Probabilistic synthesis
    LDIPS               // SOTA deterministic synthesis
};

enum SYNTHESIS_SETTING {
    FULL,               // SOTA synthesis
    INCREMENTAL         // Incremental synthesis that builds off the previous best program
};

LABELER labeler = STABLE_HEURISTIC;
SYNTHESIS_ALGO synthesizer = EMDIPS;
SYNTHESIS_SETTING synth_setting = INCREMENTAL;