include(FindPackageHandleStandardArgs)

find_library(Z3_LIBRARIES
  NAMES z3
  DOC "Z3 libraries")

find_path(Z3_INCLUDE_DIRS
  NAMES z3++.h
  DOC "Z3 C++ API header")

find_package_handle_standard_args(Z3 DEFAULT_MSG Z3_INCLUDE_DIRS Z3_LIBRARIES)