#!/bin/bash



# generate observations and low-level actions
mkdir -p accSim/out
g++ -o accSim/out/gen accSim/generate.cpp -std=c++14
# ./gen <robot test set> <model> <mean error> <error std dev> <high-level success rate>
        # <accMax> <decMax> <maxSpeed> <targetDistance> (optional; pass in iff robot test set=3)
./accSim/out/gen 3 0 0.0 0.1 0.9 6 -5 15 100


# generate high-level actions from observations and low-level actions
# ./pf <accMax> <decMax> <maxSpeed> <targetDistance> <mean error> <error std dev> <high-level success rate>
        # <model> <numParticles> <resampleThreshold> <LA std dev>
g++ -o pf_custom/out/pf pf_custom/pf_runner.cpp -std=c++14
pf_custom/out/pf 6 -5 15 100 0.0 0.2 1 2 1000 0.1 8.0

# resamplingThreshold range with these settings: 0.002 to 0.01
# TODO: add images of varying thresholds
# TODO: instead of printing "resamp" everywhere, just print out number of times resampled
# never resamples at 0.001 for some reason, maybe bc .001 * 1000 = 1


# python3 plotter.py <max time steps> <particles to plot>
python3 pf_custom/plotter.py 1000 10