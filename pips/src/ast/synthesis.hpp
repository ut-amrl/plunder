// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <z3++.h>
#include "Python.h"

#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "ast.hpp"

namespace AST {

    struct EmdipsOutput {
        std::vector<ast_ptr> ast_vec;
        std::vector<float> log_likelihoods;
    };

    ast_ptr PredicateL2(const std::vector<Example>& examples,
        const std::vector<ast_ptr>& ops,
        ast_ptr sketch,
        const std::pair<std::string, std::string>& transition,
        const double min_accuracy, float* best_score);

    ast_ptr ldipsL2(ast_ptr candidate,
        const std::vector<Example>& examples,
        const std::vector<ast_ptr>& ops,
        const std::pair<std::string, std::string>& transition,
        const float min_accuracy,
        ast_ptr best_program,
        float* best_score);

    std::vector<ast_ptr> ldipsL3(const std::vector<Example>& demos,
        const std::vector<std::pair<std::string, std::string>>& transitions,
        const std::vector<ast_ptr> lib,
        const int sketch_depth,
        const float min_accuracy,
        const std::string& output_path);

    ast_ptr LikelihoodPredicateL2(const std::vector<Example>& examples,
        const std::vector<ast_ptr>& ops,
        ast_ptr sketch,
        const std::pair<std::string, std::string>& transition,
        const double max_error, float* best_error);
    
    std::pair<ast_ptr, float> emdipsL2(ast_ptr candidate,
        const std::vector<Example>& examples,
        const std::vector<ast_ptr>& ops,
        const std::pair<std::string, std::string>& transition,
        const float max_error);

    EmdipsOutput emdipsL3(const std::vector<Example>& demos,
        const std::vector<std::pair<std::string, std::string>>& transitions,
        const std::vector<ast_ptr> lib,
        const int sketch_depth,
        const std::vector<float> max_error,
        const std::string& output_path,
        const uint32_t b_size,
        PyObject* optimizer);

    void SRTR(const std::vector<Example>& demos,
        const std::vector<ast_ptr>& programs,
        const std::vector<std::pair<std::string, std::string>>& transitions,
        const std::string& output_path);

    void DIPR(const std::vector<Example>& demos,
        const std::vector<ast_ptr>& programs,
        const std::vector<std::pair<std::string, std::string>>& transitions,
        const std::vector<ast_ptr> lib,
        const int sketch_depth,
        const float min_accuracy,
        const std::string& output_path);

}  // namespace AST
