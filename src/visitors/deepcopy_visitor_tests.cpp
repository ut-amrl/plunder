#include <gtest/gtest.h>

#include <memory>

#include "../ast.hpp"
#include "deepcopy_visitor.hpp"

using AST::ast_ptr;
using AST::bin_ptr;
using AST::BinOp;
using AST::DeepCopyAST;
using AST::Num;
using AST::num_ptr;
using std::dynamic_pointer_cast;
using std::make_shared;

TEST(DeepCopyVisitor, basically_works) {
  Num lhs(4.0, {1, 2, 3});
  Num rhs(5.0, {4, 5, 6});
  num_ptr lhs_ptr = make_shared<Num>(lhs);
  num_ptr rhs_ptr = make_shared<Num>(rhs);

  BinOp add(lhs_ptr, rhs_ptr, "Add");
  bin_ptr add_ptr = make_shared<BinOp>(add);

  ast_ptr result = DeepCopyAST(add_ptr);
  bin_ptr new_add_ptr = dynamic_pointer_cast<BinOp>(result);
  num_ptr new_lhs_ptr = dynamic_pointer_cast<Num>(new_add_ptr->left_);
  num_ptr new_rhs_ptr = dynamic_pointer_cast<Num>(new_add_ptr->right_);

  // Check that the pointers themselves are different.
  ASSERT_NE(add_ptr, result);
  ASSERT_NE(add_ptr, new_add_ptr);
  ASSERT_NE(add_ptr->left_, new_add_ptr->left_);
  ASSERT_NE(add_ptr->right_, new_add_ptr->right_);

  // Check that the pointees are the same.
  ASSERT_EQ(add_ptr->op_, new_add_ptr->op_);
  ASSERT_EQ(lhs_ptr->value_, new_lhs_ptr->value_);
  ASSERT_EQ(lhs_ptr->dims_, new_lhs_ptr->dims_);
  ASSERT_EQ(rhs_ptr->value_, new_rhs_ptr->value_);
  ASSERT_EQ(rhs_ptr->dims_, new_rhs_ptr->dims_);
}

TEST(DeepCopyVisitor, nested_binop) {
  Num lhs1(4.0, {1, 2, 3});
  Num rhs1(5.0, {4, 5, 6});
  num_ptr lhs1_ptr = make_shared<Num>(lhs1);
  num_ptr rhs1_ptr = make_shared<Num>(rhs1);

  Num lhs2(6.0, {3, 2, 1});
  Num rhs2(7.0, {6, 5, 4});
  num_ptr lhs2_ptr = make_shared<Num>(lhs2);
  num_ptr rhs2_ptr = make_shared<Num>(rhs2);

  BinOp lt(lhs1_ptr, rhs1_ptr, "Lt");
  BinOp gt(lhs2_ptr, rhs2_ptr, "Gt");
  bin_ptr lt_ptr = make_shared<BinOp>(lt);
  bin_ptr gt_ptr = make_shared<BinOp>(gt);

  BinOp and_(lt_ptr, gt_ptr, "And");
  bin_ptr and_ptr = make_shared<BinOp>(and_);

  ast_ptr result = DeepCopyAST(and_ptr);
  bin_ptr new_and_ptr = dynamic_pointer_cast<BinOp>(result);
  bin_ptr new_lt_ptr = dynamic_pointer_cast<BinOp>(new_and_ptr->left_);
  bin_ptr new_gt_ptr = dynamic_pointer_cast<BinOp>(new_and_ptr->right_);
  ASSERT_EQ(and_ptr->op_, new_and_ptr->op_);
  ASSERT_EQ(lt_ptr->op_, new_lt_ptr->op_);
  ASSERT_EQ(gt_ptr->op_, new_gt_ptr->op_);
}