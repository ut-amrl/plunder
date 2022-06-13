#!/bin/bash

cd ../accSim
g++ -o gen generate.cpp -std=c++14
# ./gen <robot test set> <model> <mean error> <error std dev> <high-level success rate>
        # <accMax> <decMax> <maxSpeed> <targetDistance> (optional; pass in iff robot test set=3)
./gen 3 2 0.0 0.2 0.8 6 -5 15 100

cd ../pf_custom
# ./pf <accMax> <decMax> <maxSpeed> <targetDistance> <mean error> <error std dev> <high-level success rate>
        # <model> <numParticles> <resampleThreshold> <LA std dev>
g++ -o pf pf_runner.cpp -std=c++14
./pf 6 -5 15 100 0.0 0.2 0.8 0 100 0.5 10.0

# python3 plotter.py <max time steps> <particles to plot>
python3 plotter.py 1000 10