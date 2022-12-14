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
#include <cmath>
#include <cassert>
#include <numeric>
#include <math.h>
#include <cstring>

#include <dlfcn.h>
#include <z3++.h>

using namespace std;

// Robot parameters
const int robotTestSet = 0;         // which robots to use (0-2)
const int numRobots = 11;           // number of robots (depends on robot test set)
const int model = 1;                // which ASP to use
const double meanError = 0.0;       // low-level action error
const double stddevError = 0.1;     // low-level action error standard deviation
const double laChangeSpeed = 2;     // rate of change of acceleration (jerk)
const double switchingError = 0.3;  // additional low-level action error standard deviation while transitioning
const double epsilon = 10E-10;
const int precision = 10;

// I/O parameters
const string stateGenPath = "accSim/out/data";                  // Generated data from simulation, contains HA, LA, and observed state sequences
const string aspPathBase = "synthesis/out/asp";                 // ASPs generated by EMDIPS
const string trajGenPath = "synthesis/out/examples/pf";         // Trajectories (high-level action sequences) generated by particle filter
const string altPath = "synthesis/out/states/iter";             // Trajectories generated from ASPs
const string operationLibPath = "pips/ops/emdips_test.json";    // Operation library for EMDIPS
const string plotGenPath = "synthesis/plots/";                  // Plots
const string gt_asp = "synthesis/gt_asp/";                      // Ground truth ASP

// Simulation parameters
const double T_STEP = .1;               // time step (s)
const double T_TOT = 15;                // total time (s) per simulated scenario
const double genAccuracy = 1.0;         // probability of a correct high-level transition

// EM Loop parameters
const int numIterations = 10;             // number of iterations in the expectation-maximization loop
const int sampleSize = 10;               // number of trajectories to process then pass into EMDIPS, per robot
const bool usePointError = true;          // point error: random transitions to a new high-level action
const double pointAccuracy = 0.95;        // probability of a correct (ASP-consistent) high-level transition
const int structuralChangeFrequency = 1;  // only enumerate over new program structures every n iterations, else tune parameters for previous best structure
const bool hardcode_program = false;      // if true then only consider single hardcoded program structure

// Optimization parameters
// TODO: move to here; find a way to efficiently retrieve them from here to the Python script

// EMDIPS parameters
const int window_size = -1;                       // Size of sampling window. -1 for n/a
const int feature_depth = 3;                      // Feature depth [using variables like v, vmax]
const int sketch_depth = 2;                       // Number of conjunctions/disjunctions
const float max_error = 0.3;                      // Target loss threshold to stop enumeration early
const int batch_size = 8;                         // Number of programs to optimize in parallel
const int programs_enumerated = 7;               // Number of programs to enumerate and optimize per iteration
const bool useSafePointError = false;             // "safe" transitions (only allow user-specified transitions)

// Plot parameters
const int particlesPlotted = 100;                   // Number of particles plotted
const int timeStepsPlot = 150;                      // Maximum time step plotted

// Particle filter parameters
const int numParticles = 20000;                                 // number of particle trajectories created to represent the distribution
const int numTrajectories = max(sampleSize, particlesPlotted);  // number of particle trajectories sampled to be fed into the maximization step
const float resampleThreshold = 1.0;                            // higher = more resampling
const float obsLikelihoodStrength = 2;                          // lower = stricter observation likelihood
const int end_pf_err = 0;                                       // ignores last n timesteps because they didn't have a chance to get resampled
const bool useSimplifiedMotorModel = false;                     // Use simulation motor model or a simplified version
const bool useSmoothTrajectories = false;                       // "Smooth" trajectories by removing single outlier timesteps
