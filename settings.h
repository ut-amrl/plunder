#pragma once

#include <string>

using namespace std;

// Robot parameters
const int robotTestSet = 3;         // which robots to use (1-3)
const int numRobots = 1;            // number of robots (depends on robot test set)
const int model = 0;                // which ASP to use
const double meanError = 0.0;       // low-level action error
const double stddevError = 2.0;     // low-level action error standard deviation
const double haProbCorrect = 0.9;   // probability of a correct high-level transition

// I/O parameters
const string stateGenPath = "accSim/out/data";                  // Generated data from simulation, contains HA, LA, and observed state sequences
const string aspPathBase = "synthesis/out/asp";                 // ASPs generated by LDIPS
const string trajGenPath = "synthesis/out/examples/pf";             // Trajectories (high-level action sequences) generated by particle filter
const string operationLibPath = "pips/ops/test_library.json";   // Operation library for LDIPS
const string plotGenPath = "synthesis/plots/";

// Simulation parameters
const double T_STEP = .1;               // time step (s)
const double T_TOT = 15;                // total time (s) per simulated scenario

// EM Loop parameters
const int numIterations = 10;           // number of iterations in the expectation-maximization loop
const int sampleSize = 20;              // number of trajectories to pass into LDIPS

// LDIPS parameters
const int window_size = 0;
const int feature_depth = 3;
const int sketch_depth = 3;
const float min_accuracy = 0.5;

// Particle filter parameters
const int numParticles = 1000;          // number of particle trajectories created to represent the distribution
const int numTrajectories = 10;         // number of particle trajectories sampled to be fed into the maximization step
const float resampleThreshold = 0.2;    // higher = more resampling

// Plot parameters
const int numParticlesPlot = 1000;
const int timeStepsPlot = 1000;