// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <ostream>

#include "ast/ast.hpp"

namespace AST {

class Print : public Visitor {
 public:
  ast_ptr Visit(AST* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Bool* node);
  ast_ptr Visit(Feature* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);
  std::string GetString() const;
  void Display();

 private:
  std::string program_ = "";
  int depth_ = 0;
};

}  // namespace AST

namespace std {

ostream& operator<<(ostream& os, const AST::ast_ptr& ast);

}  // namespace std
