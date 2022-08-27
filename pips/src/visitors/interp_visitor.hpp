// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "ast/ast.hpp"


void setBoundaryStddev(double err);
void setInterpOpt(int mode);
void setExampleState(bool yes);

namespace AST {

ast_ptr Interpret(const ast_ptr& program);
ast_ptr Interpret(const ast_ptr& program, const Example& example);
bool InterpretBool(const ast_ptr& program, const Example& example);

class Interp : public Visitor {
 public:
  Interp();
  Interp(const Example& world);
  ast_ptr Visit(AST* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Bool* node);
  ast_ptr Visit(Feature* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);

 private:
  Example world_;
};

}  // namespace AST
