# include $(shell rospack find mk)/cmake.mk
PY = python3

#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

SETTINGS = settings
GEN = gen
PF = pf
PLT = plt
EM = em
EMNG = emng
EMTEST = emtest

.SILENT:

all: build build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build

$(SETTINGS):
			./bin/settings

$(GEN):
			$(MAKE) $(SETTINGS) && \
			mkdir -p accSim/out && \
			./bin/gen && \
			cp accSim/out/data.json pips/examples/data.json

$(PF):
			$(MAKE) $(SETTINGS) && \
			mkdir -p particleFilter/out && \
			mkdir -p synthesis/out/examples && \
			./bin/pf

$(PLT):
			$(MAKE) $(SETTINGS) && \
			mkdir -p synthesis/plots && \
			mkdir -p synthesis/plots/pure && \
			mkdir -p synthesis/plots/pf && \
			$(PY) particleFilter/plotter.py

$(EMNG):
			$(MAKE) $(SETTINGS) && \
			rm -rf synthesis/out && \
			mkdir -p synthesis/out/examples && \
			mkdir -p synthesis/out/states && \
			./bin/emloop
			
$(EM):
			$(MAKE) clear_data && \
			$(MAKE) $(GEN) && \
			$(MAKE) $(EMNG)

clean:
	rm -rf bin build lib

clear_data:
	rm -rf accSim/out particleFilter/out particleFilter/plots synthesis/plots synthesis/out settings.txt

purge: clean clear_data
