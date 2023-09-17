#pragma once

#include "includes.h"

using namespace std;

namespace SETTINGS {

    // ----- I/O parameters -----
    const string SIM_DATA = "python-gen/data";          // Generated data from simulation, contains HA, LA, and observed state sequences
                                                        // Default: sim/data, is where our provided generator will store these files
    
    // ----- General configuration -----
    const bool DEBUG = false;
    const int TRAINING_SET = 5;                // Number of robots to train on
    const int VALIDATION_SET = 30;              // Number of total robots in the full set (including the training set)
    const bool USE_SAFE_TRANSITIONS = false;    // "Safe" transitions (only allow user-specified transitions). 
                                                // Optional modification to improve performance on policies with sparse transitions
    // ----- Simulation parameters -----
    const double T_STEP = -1;                   // Time step (s)
    const double T_TOT = -1;                    // Total time (s) per simulated scenario
    const double GEN_ACCURACY = -1;             // Probability of a correct high-level transition in the simulation

    // ----- EM Loop parameters -----
    const int NUM_ITER = 20;                    // Number of iterations in the expectation-maximization loop

    // ----- Parallelization Parameters -----
    const int BATCH_SIZE = 4;                  // Number of programs to optimize in parallel
    const int NUM_CORES = 4;                    // Number of cores to use per program: NUM_CORES * BATCH_SIZE = total number of cores used at once

    // ----- Important hyperparameters -----
    const double PROG_COMPLEXITY_LOSS_BASE = 0.002;     // adds L1 loss ( AST.size * PROG_COMPLEXITY_LOSS )
    const double PROG_COMPLEXITY_LOSS = 0.03;          // adds L1 loss ( AST.size * PROG_COMPLEXITY_LOSS * loss )
    const double ALPHA_LOSS_UPPER = 0.0001;             // adds L2 loss ( alpha^2 * ALPHA_LOSS_UPPER )
}