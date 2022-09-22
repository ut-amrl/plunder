// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "ast/ast.hpp"
#include "ast/enumeration.hpp"

namespace AST {

std::unordered_map<std::string, std::pair<Type, Dimension>> MapFeatureHoles(
    const ast_ptr& ast);
void ResetParams(ast_ptr ast);
int FillHoles(ast_ptr& ast, const Model& model);
ast_ptr Srtrize(ast_ptr& ast);
bool IsRelative(ast_ptr& ast);
ast_ptr FillHoles(const ast_ptr& ast, const Model& model);

class MapHoles : public Visitor {
 public:
  MapHoles();
  ast_ptr Visit(AST* node);
  ast_ptr Visit(TernOp* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Bool* node);
  ast_ptr Visit(Feature* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);
  std::unordered_map<std::string, std::pair<Type, Dimension>> GetFeatureHoles()
      const;
  std::unordered_set<std::string> GetParameterHoles() const;
  bool IsRelative() const;
  bool reset_params_;
  bool srtrize_;
  void Reset();

 private:
  std::unordered_map<std::string, std::pair<Type, Dimension>> features_;
  std::unordered_set<std::string> parameters_;
  int depth_ = 0;
  bool is_relative_ = false;
};

class FillHole : public Visitor {
 public:
  FillHole(const std::string& target_name, const ast_ptr& new_value);
  ast_ptr Visit(AST* node);
  ast_ptr Visit(TernOp* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Bool* node);
  ast_ptr Visit(Feature* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);

 private:
  const std::string target_name_;
  const ast_ptr new_value_;
};

}  // namespace AST
