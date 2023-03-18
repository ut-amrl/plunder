#pragma once

#include "includes.h"

using namespace std;

namespace SETTINGS {

    // I/O parameters
    const string SIM_DATA = "python-gen/data";                         // Generated data from simulation, contains HA, LA, and observed state sequences
    
    // General Configuration
    const bool DEBUG = false;
    const int TRAINING_SET = 10;
    const int VALIDATION_SET = 30;
    const bool USE_SAFE_TRANSITIONS = false;          // "safe" transitions (only allow user-specified transitions)

    // Simulation parameters
    const double T_STEP = -1;               // time step (s)
    const double T_TOT = -1;                // total time (s) per simulated scenario
    const double GEN_ACCURACY = -1;        // probability of a correct high-level transition in the simulation

    // EM Loop parameters
    const int NUM_ITER = 20;                    // number of iterations in the expectation-maximization loop

    // Optimization parameters
    const int BATCH_SIZE = 16;               // Number of programs to optimize in parallel
    const int NUM_CORES = 8;                // Number of cores to use per program: NUM_CORES * BATCH_SIZE = total number of cores used at once

    const double PROG_COMPLEXITY_LOSS_BASE = 0.003;     // adds L1 loss ( AST.size * PROG_COMPLEXITY_LOSS )
    const double PROG_COMPLEXITY_LOSS = 0.085;           // adds L1 loss ( AST.size * PROG_COMPLEXITY_LOSS * loss)
    const double ALPHA_LOSS_UPPER = 0.0001;           // adds L2 loss ( alpha^2 * ALPHA_LOSS_UPPER )                        // TEMPERATURE decreases linearly by this much each iteration
}