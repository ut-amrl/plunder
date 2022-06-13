#!/bin/bash



# generate observations and low-level actions
mkdir -p accSim/out
g++ -o accSim/out/gen accSim/generate.cpp -std=c++14
# ./gen <robot test set> <model> <mean error> <error std dev> <high-level success rate>
        # <accMax> <decMax> <maxSpeed> <targetDistance> (optional; pass in iff robot test set=3)
./accSim/out/gen 3 0 0.0 0.2 0.8 6 -5 15 100


# generate high-level actions from observations and low-level actions
cd pf_custom
# ./pf <accMax> <decMax> <maxSpeed> <targetDistance> <mean error> <error std dev> <high-level success rate>
        # <model> <numParticles> <resampleThreshold> <LA std dev>
g++ -o pf pf_runner.cpp -std=c++14
./pf 6 -5 15 100 0.0 0.2 0.8 2 1000 0.001 10.0


# python3 plotter.py <max time steps> <particles to plot>
python3 plotter.py 1000 10