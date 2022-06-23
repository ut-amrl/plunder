#pragma once

#include <string>

using namespace std;

// Robot parameters
const int robotTestSet = 0;         // which robots to use (0-2)
const int numRobots = 4;            // number of robots (depends on robot test set)
const int model = 0;                // which ASP to use
const double meanError = 0.0;       // low-level action error
const double stddevError = 1.0;     // low-level action error standard deviation

// I/O parameters
const string stateGenPath = "accSim/out/data";                  // Generated data from simulation, contains HA, LA, and observed state sequences
const string aspPathBase = "synthesis/out/asp";                 // ASPs generated by EMDIPS
const string trajGenPath = "synthesis/out/examples/pf";         // Trajectories (high-level action sequences) generated by particle filter
const string operationLibPath = "pips/ops/test_library.json";   // Operation library for EMDIPS
const string plotGenPath = "synthesis/plots/";

// Simulation parameters
const double T_STEP = .1;               // time step (s)
const double T_TOT = 15;                // total time (s) per simulated scenario
const double genAccuracy = 1.0;  // probability of a correct high-level transition

// EM Loop parameters
const int numIterations = 10;           // number of iterations in the expectation-maximization loop
const int sampleSize = 20;              // number of trajectories to pass into EMDIPS
const bool usePointError = true;        // point error: random transitions to a new high-level action
const double pointAccuracy = 0.8;       // probability of a correct (ASP-consistent) high-level transition
const bool useBoundaryError = true;    // boundary error: threshold-dependent error in EMDIPS-generated ASP
const double boundaryDeviation = 5.0;   // conditional deviation

// EMDIPS parameters
const int window_size = 4;
const int feature_depth = 3;
const int sketch_depth = 2;
const float min_accuracy = 0.95;               // Only when running EMDIPS and not the full EM loop

// Plot parameters
const int particlesPlotted = 10;
const int timeStepsPlot = 1000;

// Particle filter parameters
const int numParticles = 2000;                                  // number of particle trajectories created to represent the distribution
const int numTrajectories = max(sampleSize, particlesPlotted);  // number of particle trajectories sampled to be fed into the maximization step
const float resampleThreshold = 0.1;                            // higher = more resampling
const double pf_stddevError = 2.0;
