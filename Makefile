CC = g++
CFLAGS = -g -std=c++17 -march=native -ggdb -O2 -fPIC -fopenmp #-Wall
PY = python3

TARGET1 = gen
TARGET2 = ldips
TARGET3 = pf
TARGET4 = plt
TARGET5 = makepips
TARGET6 = em

PIPS_OPTIONS = -min_accuracy 0.8 -debug true -window_size 0


.SILENT: $(TARGET1) $(TARGET2) $(TARGET3) $(TARGET4) $(TARGET5) $(TARGET6) all more clean


$(TARGET1):
			mkdir -p accSim/out && \
			$(CC) $(CFLAGS) -o accSim/out/gen accSim/generate.cpp && \
			accSim/out/gen 3 0 0.0 0.0 1.0 && \
			cp accSim/out/data.json pips/examples/data.json

$(TARGET2):
			pips/bin/ldips-l3 $(PIPS_OPTIONS) -lib_file pips/ops/test_library.json -ex_file pips/examples/data.json

$(TARGET3):
			mkdir -p particleFilter/out && \
			$(CC) $(CFLAGS) -o particleFilter/out/pf particleFilter/pf_runner.cpp && \
			particleFilter/out/pf 3 0 1 0.9 5 1000 0.2

$(TARGET4):
			mkdir -p particleFilter/plots && \
			$(PY) particleFilter/plotter.py 1000 1000 10

$(TARGET5):
			cd pips && make && cd ..

$(TARGET6):
			mkdir -p synthesis/out/examples && \
			$(CC) $(CFLAGS) synthesis/em.cpp -L pips/lib \
			-l c++-pips-core -l amrl_shared_lib -l z3 \
			-o synthesis/out/em \
			-I pips/src -I pf_custom -I accSim -I pips/submodules/json/single_include/ && \
			synthesis/out/em


all: $(TARGET1) $(TARGET3) $(TARGET4)

clean: 
			rm -rf accSim/out \
					particleFilter/out \
					particleFilter/plots \
					synthesis/out
