#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <random>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <assert.h>
#include <stdlib.h>
#include <filesystem>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <set>
#include <cmath>
#include <cassert>
#include <numeric>
#include <math.h>
#include <cstring>
#include <optional>

#include <dlfcn.h>
#include <z3++.h>

#include "ast/ast.hpp"
#include "ast/enumeration.hpp"
#include "ast/library_functions.hpp"
#include "ast/parsing.hpp"
#include "visitors/interp_visitor.hpp"
#include "ast/synthesis.hpp"

typedef int HA;

const string OPTIMIZER_PATH = "../pips/src/optimizer";  // Path to optimizer (for Python support)
const string OPERATION_LIB = "../pips/ops/emdips_operations.json"; // Path to operation library