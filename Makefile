CC = g++
CFLAGS = -g -Wall -std=c++17

TARGET1 = gen
TARGET2 = syn

PIPS_OPTIONS = -min_accuracy 0.5 -debug true -window_size 0


$(TARGET1):
			mkdir -p accSim/out && \
			$(CC) $(CFLAGS) -o accSim/out/gen accSim/generate.cpp && \
			accSim/out/gen

$(TARGET2):
			pips/bin/ldips-l3 $(PIPS_OPTIONS) -lib_file pips/ops/test_library.json -ex_file pips/examples/data.json

clean: 
			rm -rf accSim/out
