CC = g++
CFLAGS = -g -Wall -std=c++17
PY = python3

TARGET1 = gen
TARGET2 = syn
TARGET3 = pf
TARGET4 = plt

PIPS_OPTIONS = -min_accuracy 0.5 -debug true -window_size 0


.SILENT: $(TARGET1) $(TARGET2) $(TARGET3) $(TARGET4) all more clean


$(TARGET1):
			mkdir -p accSim/out && \
			$(CC) $(CFLAGS) -o accSim/out/gen accSim/generate.cpp && \
			accSim/out/gen

$(TARGET2):
			pips/bin/ldips-l3 $(PIPS_OPTIONS) -lib_file pips/ops/test_library.json -ex_file pips/examples/data.json

$(TARGET3):
			mkdir -p pf_custom/out && \
			$(CC) $(CFLAGS) -o pf_custom/out/pf pf_custom/pf_runner.cpp && \
			pf_custom/out/pf

$(TARGET4):
			mkdir -p pf_custom/plots && \
			$(PY) pf_custom/plotter.py


all: $(TARGET1) $(TARGET3) $(TARGET4)
more: $(TARGET1) $(TARGET2) $(TARGET3) $(TARGET4)


clean: 
			rm -rf accSim/out \
					pf_custom/out \
					pf_custom/plots
