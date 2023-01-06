#pragma once

#include "includes.h"

using namespace std;

namespace SETTINGS {

    // I/O parameters
    const string SIM_DATA = "sim/data";                  // Generated data from simulation, contains HA, LA, and observed state sequences
    const string GEN_ASP = "out/asp_iter";                 // ASPs generated by EMDIPS
    const string PF_TRAJ = "out/pf_traj/iter";         // Trajectories (high-level action sequences) generated by particle filter
    const string PURE_TRAJ = "out/pure_traj/iter";       // Trajectories generated from ASPs
    const string PLOT_PATH = "plots/";                // Plots
    const string GT_ASP_PATH = "gt_asp/";             // Ground truth ASP

    // General Configuration
    const bool DEBUG = false;
    const double EPSILON = 10E-10;
    const int PRECISION = 10;
    const int NUM_ROBOTS = 11;              // number of robots (depends on robot test set)

    // Simulation parameters
    const double T_STEP = .1;               // time step (s)
    const double T_TOT = 15;                // total time (s) per simulated scenario
    const double GEN_ACCURACY = 1.0;        // probability of a correct high-level transition

    // EM Loop parameters
    const int NUM_ITER = 10;                    // number of iterations in the expectation-maximization loop
    const int SAMPLE_SIZE = 10;                 // number of trajectories to process then pass into EMDIPS, per robot
    bool USE_POINT_ERROR = true;                // point error: random transitions to a new high-level action
    const double POINT_ACCURACY = 0.9;          // probability of a correct (ASP-consistent) high-level transition
    const int STRUCT_CHANGE_FREQ = 1;           // only enumerate over new program structures every n iterations, else tune parameters for previous best structure

    // Plot parameters
    const bool GT_PRESENT = true;
    const int PARTICLES_PLOTTED = 100;                                  // Number of particles plotted
    const int PLOT_TIME = 150;                                          // Maximum time step plotted
    const int NUM_TRAJECTORIES = max(SAMPLE_SIZE, PARTICLES_PLOTTED);   // number of particle trajectories sampled to be fed into the maximization step

    // Optimization parameters
    const int OPT_METHOD = 1;                  // optimization method, See below
    const bool ENUMERATE_SIGNS = true;         // Equivalent to enumerating over > and <
    const bool PRINT_DEBUG = false;            // Extra debugging info
    const int INITIAL_VALUES = 8;              // Initial values for x_0: 0 = all zeros, 1 = average, >1 = do all of the above, then enumerate over random initial guesses (use this to specify how many)
    const int NUM_CORES = 4;                   // Number of processes to run in parallel
    const double MIN_ALPHA = 1.0;              // lowest slope allowed
    const double INIT_ALPHA = 1.0;             // starting slope
    const bool BOUND_ALPHA = false;             // whether to bound alpha (to ensure slope is not too low)
    const double BOUNDS_EXTEND = 0.1;          // Amount to search above and below extrema
    const bool PRINT_WARNINGS = false;         // Debugging info
    const int PRINT_PADDING = 30;              // Print customization
    const double OUTLIER_MAX = 20;             // Max negative log likelihood that an example can contribute to the total log likelihood
    const bool BOUND_LIKELIHOOD = false;       // Whether we bound the likelihood by tt
    const int MAX_ITER = 150;                  // Max number of iterations of a single optimization run

    const double PROG_COMPLEXITY_LOSS = 0.0;      // adds L1 loss ( num_parameters * PROG_COMPLEXITY_LOSS )
    const double ALPHA_LOSS_LOWER = 20.0;          // adds L2 loss ( 1/alpha^2 * ALPHA_LOSS_LOWER )
    const double ALPHA_LOSS_UPPER = 0.001;         // adds L2 loss ( alpha^2 * ALPHA_LOSS_UPPER )
    const double X_0_LOSS = 0;                    // adds L1 loss ( x_0 * X_0_LOSS )

    const int MAX_EX_YES = 50;         // Total number of examples to optimize over when transition is satisfied
    const int MAX_EX_NO = 200;         // Total number of examples to optimize over when a transition is not satisfied

    // OPT_METHOD = optimization method
    // 0: local (BFGS)
    // 1: local (L-BFGS-B)
    // 2: basin hopping
    // 3: dual annealing
    // 4: DIRECT

    // EMDIPS parameters
    const int WINDOW_SIZE = -1;                       // Size of sampling window. -1 for n/a
    const int FEATURE_DEPTH = 3;                      // Feature depth [using variables like v, vmax]
    const int SKETCH_DEPTH = 2;                       // Number of conjunctions/disjunctions
    const float TARGET_LOSS = 0.3;                    // Target loss threshold to stop enumeration early
    const int BATCH_SIZE = 8;                         // Number of programs to optimize in parallel
    const int PROG_ENUM = 7;                          // Number of programs to enumerate and optimize per iteration
    const bool USE_SAFE_TRANSITIONS = false;          // "safe" transitions (only allow user-specified transitions)
    const bool HARDCODE_PROG = false;                 // if true then only consider single hardcoded program structure

    // Particle filter parameters
    const int NUM_PARTICLES = 20000;                                 // number of particle trajectories created to represent the distribution
    const float RESAMPLE_THRESHOLD = 1.0;                            // higher = more resampling
    const float OBS_LIKELIHOOD_STRENGTH = 2;                         // lower = stricter observation likelihood
    const int END_PF_ERROR = 0;                                      // ignores last n timesteps because they didn't have a chance to get resampled
    const bool SMOOTH_TRAJECTORIES = false;                          // "Smooth" trajectories by removing single outlier timesteps

}