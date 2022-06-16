#!/bin/bash

# generate observations and low-level actions
mkdir -p accSim/out
g++ -o accSim/out/gen accSim/generate.cpp -std=c++14
# ./gen <robot test set> <model> <mean error> <error std dev> <high-level success rate>
./accSim/out/gen 3 1 0.0 1.0 0.9


# generate high-level actions from observations and low-level actions
mkdir -p particleFilter/out
# ./pf <accMax> <decMax> <maxSpeed> <targetDistance> <mean error> <error std dev> <high-level success rate>
        # <model> <numParticles> <resampleThreshold>
g++ -o particleFilter/out/pf particleFilter/pf_runner.cpp -std=c++14
particleFilter/out/pf 5 -4 12 100 0.0 2.0 0.9 4 5000 0.001

# resamplingThreshold range with these settings: 0.002 to 0.01
# TODO: add images of varying thresholds

mkdir -p particleFilter/plots
# python3 plotter.py <max time steps> <particles to plot>
python3 particleFilter/plotter.py 500 10