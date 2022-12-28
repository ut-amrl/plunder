# Set target directory here OR pass in through command line
target_dir ?= 1D-target

# acceptable build_types: Release/Debug/Profile
build_type := Release
# build_type := Debug

SETTINGS := settings
GEN := gen
PF := pf
PLT := plt
EM := em
EMNG := emng
EMTEST := emtest
D := $(shell date +%y.%m.%d-%H:%M:%S)

# include $(shell rospack find mk)/cmake.mk
PY := python3

.SILENT:

all: $(target_dir)/build $(target_dir)/build/CMakeLists.txt.copy
	$(info Build_type is [$(build_type)])
	$(info Target dir is [$(target_dir)])
	$(MAKE) --no-print-directory -C $(target_dir)/build

$(target_dir)/build/CMakeLists.txt.copy: $(target_dir)/build CMakeLists.txt Makefile
	cmake -DCMAKE_BUILD_TYPE=$(build_type) -DTARGET_DIR=$(target_dir) . -B$(target_dir)/build
	cp CMakeLists.txt $(target_dir)/build/CMakeLists.txt.copy

$(target_dir)/build:
	mkdir -p $(target_dir)/build

$(SETTINGS):
			cd $(target_dir) && \
			./bin/settings

$(GEN):
			$(MAKE) $(SETTINGS) && \
			cd $(target_dir) && \
			mkdir -p sim && \
			./bin/gen

$(PF):
			$(MAKE) $(SETTINGS) && \
			cd $(target_dir) && \
			mkdir -p out && \
			mkdir -p out/pf_traj && \
			./bin/pf

$(PLT):
			$(MAKE) $(SETTINGS) && \
			cd $(target_dir) && \
			rm -rf plots && \
			mkdir -p plots && \
			mkdir -p plots/pure && \
			mkdir -p plots/pf && \
			$(PY) ../particleFilter/plotter.py

$(EMNG):
			$(MAKE) $(SETTINGS) && \
			cd $(target_dir) && \
			rm -rf out && \
			mkdir -p out/pf_traj && \
			mkdir -p out/pure_traj && \
			touch out/em.txt && \
			bin/emloop | tee out/em.txt
			
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
