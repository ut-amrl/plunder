project(c++-pips)
cmake_minimum_required(VERSION 3.10.2)

# Tell CMake where it can find our custom .cmake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/pips/submodules/")

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Set some useful compile flags
set(CMAKE_CXX_FLAGS "-march=native -Wall -ggdb -O2 -fPIC -fopenmp")
if(${CXX} MATCHES "g++")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -rdynamic")
endif()

# TinyDir stuff
################################
# Add definitions

if(MSVC)
	add_definitions(-W4 -WX -wd"4996")
else()
	add_definitions(-fsigned-char -Wall -W -Wpointer-arith -Wcast-qual -Winline)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstrict-prototypes")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

# ros setup stuff
# INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# ROSBUILD_INIT()
# SET(ROS_BUILD_STATIC_LIBS true)
# SET(ROS_BUILD_SHARED_LIBS false)

# Load required third-party libraries from system
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTest REQUIRED)
# find_package(GFlags REQUIRED)
find_package(Threads REQUIRED)
find_package(Z3 REQUIRED)
find_package(PythonLibs REQUIRED)

# Set final locations for binaries
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Libraries
# SET(libs roslib roscpp rosbag boost_system pthread)
SET(libs glog amrl_shared_lib boost_system gflags
    pthread)

include_directories(pips/src/)
include_directories(accSim/)
include_directories(particleFilter/)
include_directories(synthesis/)
include_directories(./)
# Load amrl_shared_lib from submodule
add_subdirectory(pips/submodules/amrl_shared_lib/)
include_directories(pips/submodules/amrl_shared_lib/)
include_directories(pips/submodules/)
include_directories(${PYTHON_INCLUDE_DIRS})

# Load Gflags from submodule
add_subdirectory(pips/submodules/gflags)

# Load Niels Lohmann's JSON library from submodule
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(pips/submodules/json)

# Build the core of C++ PIPS as a static library, which the executables can
# link to. This enables shorter compile times since these files only need to be
# built once.
set(CXX_PIPS_SRCS
    pips/src/utils/nd_bool_array.cpp
    pips/src/visitors/deepcopy_visitor.cpp
    pips/src/visitors/deepcopy_visitor.hpp
    pips/src/visitors/fillhole_visitor.cpp
    pips/src/visitors/fillhole_visitor.hpp
    pips/src/visitors/interp_visitor.cpp
    pips/src/visitors/interp_visitor.hpp
    pips/src/visitors/print_visitor.cpp
    pips/src/visitors/print_visitor.hpp
    pips/src/visitors/tosmtlib_visitor.cpp
    pips/src/visitors/tosmtlib_visitor.hpp
    pips/src/ast/ast.cpp
    pips/src/ast/ast.hpp
    pips/src/ast/enumeration.cpp
    pips/src/ast/enumeration.hpp
    pips/src/ast/library_functions.cpp
    pips/src/ast/library_functions.hpp
    pips/src/ast/parsing.cpp
    pips/src/ast/parsing.hpp
    pips/src/ast/synthesis.cpp
    pips/src/ast/synthesis.hpp
    accSim/generate.cpp
    accSim/generate.h
    accSim/robotSets.h
    particleFilter/pf_runner.cpp
    particleFilter/pf_runner.h
    particleFilter/pf.h
    particleFilter/plotter.py
    particleFilter/pf_runner.cpp
    synthesis/em.cpp
    domain.h
    utils.h
    robot.h
    settings.h
    translateSettings.cpp)
add_library(c++-pips-core ${CXX_PIPS_SRCS})
# target_link_libraries(c++-pips-core amrl_shared_lib)
target_link_libraries(c++-pips-core Eigen3::Eigen)
target_link_libraries(c++-pips-core nlohmann_json::nlohmann_json)
target_link_libraries(c++-pips-core ${Z3_LIBRARIES})
target_link_libraries(c++-pips-core ${libs})
target_link_libraries(c++-pips-core ${PYTHON_LIBRARIES})

# Add an executable for each command
add_executable(emdips-l3 pips/src/emdips-l3.cpp)
target_link_libraries(emdips-l3 c++-pips-core stdc++fs)

add_executable(settings translateSettings.cpp)
target_link_libraries(settings c++-pips-core stdc++fs)

add_executable(pf particleFilter/pf_runner.cpp)
target_link_libraries(pf c++-pips-core stdc++fs)

add_executable(gen accSim/generate.cpp)
target_link_libraries(gen c++-pips-core stdc++fs)

add_executable(emloop synthesis/em.cpp)
target_link_libraries(emloop c++-pips-core stdc++fs)

