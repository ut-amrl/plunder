#include "interp_visitor.hpp"

#include <eigen3/Eigen/Core>  // Vector2f
#include <iostream>           // cout, endl
#include <memory>             // make_shared
#include <stdexcept>          // invalid_argument
#include <string>             // string
#include <random>             // random

#include "ast/library_functions.hpp"

#define SEED 3256+2585

using Eigen::Vector2f;
using namespace std;

// Set probabilistic boundaries
normal_distribution<double> boundaryDistr (0, 0);
default_random_engine emdipsGen(SEED);
int useOpt = 0;
bool exampleState = false;

void setBoundaryStddev(double err){
  boundaryDistr = normal_distribution<double>(0, err);
}

void setInterpOpt(int mode){
  useOpt = mode;
}

void setExampleState(bool yes){
  exampleState = yes;
}

namespace AST {

ast_ptr Interpret(const ast_ptr& program) {
  Interp interpreter;
  const ast_ptr result = program->Accept(&interpreter);
  return result;
}

ast_ptr Interpret(const ast_ptr& program, const Example& example) {
  Interp interpreter(example);
  const ast_ptr result = program->Accept(&interpreter);
  return result;
}

bool InterpretBool(const ast_ptr& program, const Example& example) {
  Interp interpreter(example);
  ast_ptr result = program->Accept(&interpreter);
  if (result->type_ == BOOL) {
    bool_ptr bool_result = std::dynamic_pointer_cast<Bool>(result);
    return bool_result->value_;
  }
  throw invalid_argument("Not a boolean expression.");
}

Interp::Interp() {}

Interp::Interp(const Example& world) : world_(world) {}

ast_ptr Interp::Visit(AST* node) { return ast_ptr(node); }

ast_ptr handleOptAnd(ast_ptr left, ast_ptr right){
  num_ptr left_cast = dynamic_pointer_cast<Num>(left);
  num_ptr right_cast = dynamic_pointer_cast<Num>(right);
  float res = exampleState ? left_cast->value_ + right_cast->value_ : fmin(left_cast->value_, right_cast->value_);
  Num rez(res, left->dims_);
  return make_shared<Num>(rez);
}

ast_ptr handleOptOr(ast_ptr left, ast_ptr right){
  num_ptr left_cast = dynamic_pointer_cast<Num>(left);
  num_ptr right_cast = dynamic_pointer_cast<Num>(right);
  float res = !exampleState ? left_cast->value_ + right_cast->value_ : fmin(left_cast->value_, right_cast->value_);
  Num rez(res, left->dims_);
  return make_shared<Num>(rez);
}

float optGtErr(float a, float b){
  // cout << a << " " << b << endl;
  // cout << pow(fmin(a-b, 0), 2) << endl;
  return pow(fmin(a-b, 0), 2);
}

float optLtErr(float a, float b){
  // cout << a << " " << b << endl;
  // cout << pow(fmax(a-b, 0), 2) << endl;
  return pow(fmax(a-b, 0), 2);
}

ast_ptr handleOptGt(ast_ptr left, ast_ptr right){
  num_ptr left_cast = dynamic_pointer_cast<Num>(left);
  num_ptr right_cast = dynamic_pointer_cast<Num>(right);
  float res = exampleState ? optGtErr(left_cast->value_, right_cast->value_) : optLtErr(left_cast->value_, right_cast->value_);
  Num rez(res, left->dims_);
  // cout << left_cast->value_ << " " << right_cast->value_ << endl;
  return make_shared<Num>(rez);
}

ast_ptr handleOptLt(ast_ptr left, ast_ptr right){
  num_ptr left_cast = dynamic_pointer_cast<Num>(left);
  num_ptr right_cast = dynamic_pointer_cast<Num>(right);
  float res = !exampleState ? optGtErr(left_cast->value_, right_cast->value_) : optLtErr(left_cast->value_, right_cast->value_);
  Num rez(res, left->dims_);
  // cout << left_cast->value_ << " " << right_cast->value_ << endl;
  return make_shared<Num>(rez);
}

ast_ptr handleOptSquareDiff(ast_ptr left, ast_ptr right){
  num_ptr left_cast = dynamic_pointer_cast<Num>(left);
  num_ptr right_cast = dynamic_pointer_cast<Num>(right);
  float res = pow(left_cast->value_ - right_cast->value_, 2);
  Num rez(res, left->dims_);
  return make_shared<Num>(rez);
}

ast_ptr handleOptSum(ast_ptr left, ast_ptr right){
  num_ptr left_cast = dynamic_pointer_cast<Num>(left);
  num_ptr right_cast = dynamic_pointer_cast<Num>(right);
  float res = left_cast->value_ + right_cast->value_;
  Num rez(res, left->dims_);
  return make_shared<Num>(rez);
}



ast_ptr Interp::Visit(BinOp* node) {
  ast_ptr left = node->left_->Accept(this);
  ast_ptr right = node->right_->Accept(this);
  const string op = node->op_;
  BinOp partial(left, right, node->op_, node->type_, node->dims_);
  ast_ptr result = make_shared<BinOp>(partial);
  // Don't try to evaluate if one of the arguments is symbolic
  if (left->symbolic_ || right->symbolic_) {
    result->symbolic_ = true;
  } else {
    // One if clause per binary operation
    if (op == "Plus") {
      // cout << "plus" << endl;
      result = Plus(left, right);
    } else if (op == "Minus") {
      // cout << "minus" << endl;
      result = Minus(left, right);
    } else if (op == "Times") {
      result = Times(left, right);
    } else if (op == "DividedBy") {
      result = DividedBy(left, right);
    } else if (op == "Cross") {
      result = Cross(left, right);
    } else if (op == "Dot") {
      result = Dot(left, right);
    } else if (op == "SqDist") {
      result = SqDist(left, right);
    } else if (op == "AngleDist") {
      result = AngleDist(left, right);
    } else if (op == "DistTraveled") {
      result = DistTraveled(left, right);
    } else if (op == "And") {
      result =  useOpt == 1 ? handleOptAnd(left, right) : 
                useOpt == 2 ? handleOptSum(left, right) : 
                And(left, right);
    } else if (op == "Or") {
      result =  useOpt == 1 ? handleOptOr(left, right) : 
                useOpt == 2 ? handleOptSum(left, right) :
                Or(left, right);
    } else if (op == "Eq") {
      result = Eq(left, right);
    } else if (op == "Gt") {
      result =  useOpt == 1 ? handleOptGt(left, right) : 
                useOpt == 2 ? handleOptSquareDiff(left, right) :
                Gt(left, right);
    } else if (op == "Lt") {
      result =  useOpt == 1 ? handleOptLt(left, right) : 
                useOpt == 2 ? handleOptSquareDiff(left, right) :
                Lt(left, right);
    } else if (op == "Gte") {
      result = Gte(left, right);
    } else if (op == "Lte") {
      result = Lte(left, right);
    } else {
      throw invalid_argument("unknown binary operation `" + op + "'");
    }
  }
  
  return result;
}

ast_ptr Interp::Visit(Bool* node) { return make_shared<Bool>(*node); }

ast_ptr Interp::Visit(Feature* node) {
  if (node->current_value_ == nullptr) {
    throw invalid_argument("AST has unfilled feature holes");
  } else {
    ast_ptr result = node->current_value_->Accept(this);
    return result;
  }
}

ast_ptr Interp::Visit(Num* node) { return make_shared<Num>(*node); }

ast_ptr Interp::Visit(Param* node) {
  if (node->current_value_ == nullptr) {
    // throw invalid_argument("AST has unfilled parameter holes");
    return make_shared<Param>(*node);
  } else {
    ast_ptr result = node->current_value_->Accept(this);
    float boundaryValue = ((Num*) (result.get()))->value_;
    boundaryValue += boundaryDistr(emdipsGen); // Add normally-distributed noise
    ((Num*) (result.get()))->value_ = boundaryValue;
    return result;
  }
}

// TODO(jaholtz) Throw errors instead of printing
ast_ptr Interp::Visit(UnOp* node) {
  ast_ptr input = node->input_->Accept(this);
  const string op = node->op_;
  UnOp partial(input, node->op_, node->type_, node->dims_);
  ast_ptr result = make_shared<UnOp>(partial);
  // Don't try to evaluate if the input is symbolic
  if (input->symbolic_) {
    result->symbolic_ = true;
  } else {
    // One if clause per unary operation
    if (op == "Abs") {
      result = Abs(input);
    } else if (op == "Sq") {
      result = Sq(input);
    } else if (op == "Cos") {
      result = Cos(input);
    } else if (op == "Sin") {
      result = Sin(input);
    } else if (op == "Heading") {
      result = Heading(input);
    } else if (op == "Angle") {
      result = Angle(input);
    } else if (op == "NormSq") {
      result = NormSq(input);
    } else if (op == "Norm") {
      result = NormSq(input);
    } else if (op == "Perp") {
      result = Perp(input);
    } else if (op == "VecX") {
      result = VecX(input);
    } else if (op == "VecY") {
      result = VecY(input);
    } else if (op == "Not") {
      result = Not(input);
    } else if (op == "StraightFreePathLength") {
      result = StraightFreePathLength(input, world_.obstacles_);
    } else {
      throw invalid_argument("unknown unary operation `" + op + "'");
    }
  }
  return result;
}

ast_ptr Interp::Visit(Var* node) {
  if (world_.symbol_table_.find(node->name_) != world_.symbol_table_.end()) {
    if (node->type_ == NUM) {
      const float value = world_.symbol_table_[node->name_].GetValue();
      Num var_value(value, node->dims_);
      return make_shared<Num>(var_value);
    } else if (node->type_ == VEC) {
      const Vector2f float_vec = world_.symbol_table_[node->name_].GetValue();
      const Vector2f value = Vector2f(float_vec.data());
      Vec vec(value, node->dims_);
      return make_shared<Vec>(vec);
    } else {
      cout << node->name_ << endl;
      cout << "Error: Variable has unhandled type." << endl;
    }
  }
  cout << node->name_ << endl;
  cout << "Error: Variable not in symbol table" << endl;
  return make_shared<Var>(*node);
}

ast_ptr Interp::Visit(Vec* node) { return make_shared<Vec>(*node); }

}  // namespace AST
