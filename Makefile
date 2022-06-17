CC = g++
CFLAGS = -g -std=c++17 -march=native -ggdb -O2 -fPIC -fopenmp -Wall
PY = python3

SETTINGS = settings
GEN = gen
LDIPS = ldips
PF = pf
PLT = plt
MPIPS = mpips
EM = em
ME = me

PIPS_OPTIONS = -min_accuracy 0.8 -debug true -window_size 0


.SILENT: $(SETTINGS) $(GEN) $(LDIPS) $(PF) $(PLT) $(MPIPS) $(EM) clean

$(SETTINGS):
			$(CC) $(CFLAGS) -o ts translateSettings.cpp
			./ts settings

$(GEN):
			mkdir -p accSim/out && \
			$(CC) $(CFLAGS) -o accSim/out/gen accSim/generate.cpp && \
			accSim/out/gen && \
			cp accSim/out/data.json pips/examples/data.json

$(LDIPS):
			pips/bin/ldips-l3 $(PIPS_OPTIONS) -lib_file pips/ops/test_library.json -ex_file pips/examples/data.json

$(PF):
			mkdir -p particleFilter/out && \
			$(CC) $(CFLAGS) -o particleFilter/out/pf particleFilter/pf_runner.cpp && \
			particleFilter/out/pf

$(PLT):
			$(MAKE) $(SETTINGS) && \
			mkdir -p synthesis/plots && \
			$(PY) particleFilter/plotter.py 10000 10000

$(MPIPS):
			cd pips && $(MAKE) && cd ..

$(EM):
			$(MAKE) $(GEN) && \
			rm -rf synthesis/out && \
			mkdir -p synthesis/out/examples && \
			$(CC) $(CFLAGS) synthesis/em.cpp -L pips/lib \
			-l c++-pips-core -l amrl_shared_lib -l z3 \
			-o synthesis/out/em \
			-I pips/src -I pf_custom -I accSim -I pips/submodules/json/single_include/ && \
			synthesis/out/em

$(ME):
			$(MAKE) $(ME)

clean: 
			rm -rf accSim/out \
					particleFilter/out \
					synthesis/plots \
					synthesis/out
