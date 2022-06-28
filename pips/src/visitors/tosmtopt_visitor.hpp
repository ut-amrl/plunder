// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "ast/ast.hpp"

namespace AST {

class ToSMTOPT : public Visitor {
 public:
  ToSMTOPT(const Example& example, bool isYes);
  ast_ptr Visit(AST* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Bool* node);
  ast_ptr Visit(Feature* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);
  std::string Get() const;
  std::unordered_set<std::string> GetFeatures() const;
  std::unordered_set<std::string> GetParams() const;
  void Reset();

 private:
  const Example example_;
  bool isYes_;
  std::string output_;
  std::unordered_set<std::string> features_;
  std::unordered_set<std::string> parameters_;
};

ToSMTOPT AstToSMTOPT(const ast_ptr& ast, const Example& example, bool isYes);

}  // namespace AST
