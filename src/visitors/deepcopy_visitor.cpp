#include "deepcopy_visitor.hpp"

#include <memory>
#include <stdexcept>

// #include "../ast.hpp"

using std::logic_error;
using std::make_shared;

namespace AST {

ast_ptr DeepCopyAST(const ast_ptr& ast) {
  if (ast == nullptr) {
    return nullptr;
  }
  DeepCopy copier;
  ast->Accept(&copier);
  return copier.GetCopy();
}

DeepCopy::DeepCopy() : copy_(nullptr) {}

ast_ptr DeepCopy::Visit(AST* node) { return ast_ptr(node); }

ast_ptr DeepCopy::Visit(BinOp* node) {
  node->left_->Accept(this);
  ast_ptr lhs = copy_;
  node->right_->Accept(this);
  ast_ptr rhs = copy_;
  BinOp copy(lhs, rhs, node->op_, node->type_, node->dims_);
  copy_ = make_shared<BinOp>(copy);
  return copy_;
}

ast_ptr DeepCopy::Visit(Bool* node) {
  Bool copy(node->value_);
  copy_ = make_shared<Bool>(copy);
  return copy_;
}

ast_ptr DeepCopy::Visit(Feature* node) {
  Feature copy(node->name_, node->dims_, node->type_);
  copy.current_value_ = DeepCopyAST(node->current_value_);
  copy_ = make_shared<Feature>(copy);
  return copy_;
}

ast_ptr DeepCopy::Visit(Num* node) {
  Num copy(node->value_, node->dims_);
  copy_ = make_shared<Num>(copy);
  return copy_;
}

ast_ptr DeepCopy::Visit(Param* node) {
  Param copy(node->name_, node->dims_, node->type_);
  copy.current_value_ = DeepCopyAST(node->current_value_);
  copy_ = make_shared<Param>(copy);
  return copy_;
}

ast_ptr DeepCopy::Visit(UnOp* node) {
  node->input_->Accept(this);
  UnOp copy(copy_, node->op_);
  copy_ = make_shared<UnOp>(copy);
  return copy_;
}

ast_ptr DeepCopy::Visit(Var* node) {
  Var copy(node->name_, node->dims_, node->type_);
  copy_ = make_shared<Var>(copy);
  return copy_;
}

ast_ptr DeepCopy::Visit(Vec* node) {
  Vec copy(node->value_, node->dims_);
  copy_ = make_shared<Vec>(copy);
  return copy_;
}

ast_ptr DeepCopy::GetCopy() const { return copy_; }

}  // namespace AST
