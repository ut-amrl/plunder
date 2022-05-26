// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "ast.hpp"
#include "enumeration.hpp"

AST::Type StringToType(const std::string& type_string);
std::string TypeToString(AST::Type type);
std::vector<AST::FunctionEntry> ReadLibrary(const std::string& file);
std::vector<AST::Example> ReadExamples(const std::string& file,
                                       std::unordered_set<AST::Var>& vars);

// Hash for using an unordered_set with pairs.
struct pair_hash {
	template <class T1, class T2>
	std::size_t operator () (std::pair<T1, T2> const &pair) const
	{
		std::size_t h1 = std::hash<T1>()(pair.first);
		std::size_t h2 = std::hash<T2>()(pair.second);

		return h1 ^ h2;
	}
};

std::vector<AST::Example> WindowExamples(
		const std::vector<AST::Example>& examples,
		const int window_size);

std::vector<AST::Example> ReadExamples(const std::string& file,
    std::unordered_set<AST::Var>& vars,
    std::vector<std::pair<std::string, std::string>>* transitions);

AST::ast_ptr LoadJson(const std::string& file);
AST::Example JsonToExample(const nlohmann::json& example);

std::vector<AST::ast_ptr> LoadSketches(const std::string& dir,
    std::vector<std::pair<std::string, std::string>>* branches);

std::vector<AST::ast_ptr> LoadSketches(const std::string& dir,
    const std::vector<std::pair<std::string, std::string>>& branches);

bool ExistsFile(const std::string& filename);
