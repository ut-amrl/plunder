// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#include "ast/ast.hpp"

namespace AST {

ast_ptr DeepCopyAST(const ast_ptr& ast);

class DeepCopy : public Visitor {
 public:
  DeepCopy();
  ast_ptr Visit(AST* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Bool* node);
  ast_ptr Visit(Feature* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);
  ast_ptr GetCopy() const;

 private:
  ast_ptr copy_;
};

}  // namespace AST
