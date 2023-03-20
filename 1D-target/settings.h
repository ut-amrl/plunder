#pragma once

#include "includes.h"

using namespace std;

namespace SETTINGS {

    // ----- I/O parameters -----
    const string SIM_DATA = "sim/data";         // Generated data from simulation, contains HA, LA, and observed state sequences
                                                // Default: sim/data, is where our provided generator will store these files
    
    // ----- General configuration -----
    const bool DEBUG = false;
    const int TRAINING_SET = 10;                // Number of robots to train on
    const int VALIDATION_SET = 30;              // Number of total robots in the full set (including the training set)
    const bool USE_SAFE_TRANSITIONS = false;    // "Safe" transitions (only allow user-specified transitions). 
                                                // Optional modification to improve performance on policies with sparse transitions

    // ----- Simulation parameters -----
    const double T_STEP = .1;                   // Time step (s)
    const double T_TOT = 12.5;                  // Total time (s) per simulated scenario
    const double GEN_ACCURACY = 1.0;            // Probability of a correct high-level transition in the simulation

    // ----- EM Loop parameters -----
    const int NUM_ITER = 20;                    // Number of iterations in the expectation-maximization loop

    // ----- Parallelization Parameters -----
    const int BATCH_SIZE = 16;                  // Number of programs to optimize in parallel
    const int NUM_CORES = 8;                    // Number of cores to use per program: NUM_CORES * BATCH_SIZE = total number of cores used at once
    
    // ----- Important hyperparameters -----

    /* 
        ### Complexity loss ###
        Penalizes complex programs proportional to the size of their AST. Overcomplex programs can indicate overfitting.

        PROG_COMPLEXITY_LOSS is used to tune the penalty. In general, complexity loss can vary widely based on how much error the program induces.
        This value takes into account the current loss, a heuristic that helps select a more accurate constant.

        PROG_COMPLEXITY_LOSS_BASE does not take into account the current loss, and provides a baseline to prevent degeneracy for low-error programs.

        If your programs are overfitting, it may help to increase PROG_COMPLEXITY_LOSS, and vice versa if they are underfitting.
        In general, we find that a value between 0.01 and 0.05 works well.
    */
    const double PROG_COMPLEXITY_LOSS_BASE = 0.005; // adds L1 loss ( AST.size * PROG_COMPLEXITY_LOSS )
    const double PROG_COMPLEXITY_LOSS = 0.03;       // adds L1 loss ( AST.size * PROG_COMPLEXITY_LOSS * loss )

    /* 
        ### Alpha loss ###
        Penalizes overconfident programs, allowing greater exploration during the E-step. Higher = less confident
        We find that a value between 0 and 0.01 works well.
    */
    const double ALPHA_LOSS_UPPER = 0.01;           // adds L2 loss ( alpha^2 * ALPHA_LOSS_UPPER )
}