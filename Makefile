# Set target directory here OR pass in through command line
target_dir ?= 1D-target

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

# include $(shell rospack find mk)/cmake.mk
PY = python3

.SILENT:

all: build build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) -DTARGET_DIR=$(target_dir) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build

$(SETTINGS):
			cp $(target_dir)/settings.h settings.h && \
			./bin/settings && \
			rm settings.h

$(GEN):
			$(MAKE) $(SETTINGS) && \
			mkdir -p $(target_dir)/sim && \
			./bin/gen

$(PF):
			$(MAKE) $(SETTINGS) && \
			mkdir -p $(target_dir)/out && \
			mkdir -p $(target_dir)/out/pf_traj && \
			./bin/pf

$(PLT):
			$(MAKE) $(SETTINGS) && \
			rm -rf $(target_dir)/plots && \
			mkdir -p $(target_dir)/plots && \
			mkdir -p $(target_dir)/plots/pure && \
			mkdir -p $(target_dir)/plots/pf && \
			$(PY) particleFilter/plotter.py

$(EMNG):
			$(MAKE) $(SETTINGS) && \
			rm -rf $(target_dir)/out && \
			mkdir -p $(target_dir)/out/pf_traj && \
			mkdir -p $(target_dir)/out/pure_traj && \
			touch $(target_dir)/out/em.txt && \
			./bin/emloop | tee $(target_dir)/out/em.txt
			
$(EM):
			$(MAKE) clear_data && \
			$(MAKE) $(GEN) && \
			$(MAKE) $(EMNG)

clean:
	rm -rf bin build lib

clear_data:
	rm -rf $(target_dir)/sim $(target_dir)/plots $(target_dir)/out settings.txt

purge: clean clear_data

snapshot:
	echo $(D)-$(FN) && \
	mkdir -p $(target_dir)/saved_outputs/$(D)-$(FN) && \
	cp -r $(target_dir)/sim $(target_dir)/plots $(target_dir)/out settings.txt pips/src/optimizer/optimizer.py saved_outputs/$D-$(FN)
