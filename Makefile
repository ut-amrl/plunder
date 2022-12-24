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
D=$(shell date +%y.%m.%d-%H:%M:%S)

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
			mkdir -p simulation/out && \
			./bin/gen

$(PF):
			$(MAKE) $(SETTINGS) && \
			mkdir -p particleFilter/out && \
			mkdir -p synthesis/out/examples && \
			./bin/pf

$(PLT):
			$(MAKE) $(SETTINGS) && \
			rm -rf synthesis/plots && \
			mkdir -p synthesis/plots && \
			mkdir -p synthesis/plots/pure && \
			mkdir -p synthesis/plots/pf && \
			$(PY) particleFilter/plotter.py

$(EMNG):
			$(MAKE) $(SETTINGS) && \
			rm -rf synthesis/out && \
			mkdir -p synthesis/out/examples && \
			mkdir -p synthesis/out/states && \
			touch synthesis/out/em.txt && \
			./bin/emloop | tee synthesis/out/em.txt
			
$(EM):
			$(MAKE) clear_data && \
			$(MAKE) $(GEN) && \
			$(MAKE) $(EMNG)

clean:
	rm -rf bin build lib

clear_data:
	rm -rf simulation/out synthesis/plots synthesis/out settings.txt

purge: clean clear_data

snapshot:
	echo $(D)-$(FN) && \
	mkdir -p saved_outputs/$(D)-$(FN) && \
	cp -r simulation/out synthesis/plots synthesis/out settings.txt pips/src/optimizer/optimizer.py saved_outputs/$D-$(FN)
