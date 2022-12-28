#pragma once

#include "includes.h"

using namespace std;

namespace SETTINGS {

    // I/O parameters
    const string SIM_DATA = "sim/data";                  // Generated data from simulation, contains HA, LA, and observed state sequences
    const string GEN_ASP = "out/asp_iter";                 // ASPs generated by EMDIPS
    const string PF_TRAJ = "out/pf_traj/iter";         // Trajectories (high-level action sequences) generated by particle filter
    const string PURE_TRAJ = "out/pure_traj/iter";       // Trajectories generated from ASPs
    const string OPERATION_LIB = "../pips/ops/emdips_test.json";   // Operation library for EMDIPS
    const string OPTIMIZER_PATH = "../pips/src/optimizer";
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
    const int PARTICLES_PLOTTED = 100;                                  // Number of particles plotted
    const int PLOT_TIME = 150;                                          // Maximum time step plotted
    const int NUM_TRAJECTORIES = max(SAMPLE_SIZE, PARTICLES_PLOTTED);   // number of particle trajectories sampled to be fed into the maximization step

    // Optimization parameters
    // TODO: move to here; find a way to efficiently retrieve them from here to the Python script

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