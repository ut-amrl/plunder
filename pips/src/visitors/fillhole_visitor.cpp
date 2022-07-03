#include "fillhole_visitor.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "ast/ast.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/tosmtlib_visitor.hpp"
#include "ast/library_functions.hpp"
#include "deepcopy_visitor.hpp"
#include "visitors/print_visitor.hpp"

using AST::Dimension;
using std::cout;
using std::endl;
using std::make_pair;
using std::make_shared;
using std::pair;
using std::string;
using std::to_string;
using std::unordered_map;
using std::unordered_set;
using std::ostream;

namespace AST {

unordered_map<string, pair<Type, Dimension>> MapFeatureHoles(
    const ast_ptr& ast) {
  MapHoles mapper;
  ast->Accept(&mapper);
  return mapper.GetFeatureHoles();
}

ast_ptr Srtrize(ast_ptr &ast) {
  MapHoles mapper;
  mapper.srtrize_ = true;
  return ast->Accept(&mapper);
}

bool IsRelative(ast_ptr &ast) {
  MapHoles mapper;
  ast->Accept(&mapper);
  return mapper.IsRelative();
}

void ResetParams(ast_ptr ast) {
  MapHoles mapper;
  mapper.reset_params_ = true;
  ast->Accept(&mapper);
}

int FillHoles(ast_ptr& ast, const Model& model) {
  // If there are no holes that still need filling, work is done so exit.
  if (model.empty()) {
    return 0;
  }

  // Otherwise, just take the first (arbitrary) hole/value pair.
  const auto& p = *(model.cbegin());
  const string& hole_name = p.first;
  const ast_ptr& hole_value = p.second;

  // Do the actual hole filling, using the visitor defined below.
  FillHole filler(hole_name, hole_value);
  ast->Accept(&filler);

  // Create a mutable copy of the model and erase the pair for the filled hole.
  unordered_map<string, ast_ptr> new_model = model;
  new_model.erase(hole_name);

  // Recursive call to fill another hole (if needed).
  return hole_value->priority + FillHoles(ast, new_model);
}

ast_ptr FillHoles(const ast_ptr& ast, const Model& model) {
  ast_ptr copy = DeepCopyAST(ast);
  FillHoles(copy, model);
  return copy;
}

MapHoles::MapHoles(): reset_params_(false),
  srtrize_(false), is_relative_(false) {
}

string PrintAst(ast_ptr ast) {
  Print printer;
  ast->Accept(&printer);
  return printer.GetString();
}

ast_ptr MapHoles::Visit(AST* node) { return ast_ptr(node); }

ast_ptr MapHoles::Visit(BinOp* node) {
  depth_ += 1;
  ast_ptr left = node->left_->Accept(this);
  ast_ptr right = node->right_->Accept(this);
  if (srtrize_) {
    BinOp srtrd(left, right, node->op_);
    node = &srtrd;
    return make_shared<BinOp>(srtrd);
  }
  is_relative_ = true;
  return make_shared<BinOp>(*node);
}

ast_ptr MapHoles::Visit(Bool* node) { return make_shared<Bool>(*node); }

ast_ptr MapHoles::Visit(Feature* node) {
  if (node->current_value_ == nullptr) {
    const string& hole_name = node->name_;
    const Type hole_type = node->type_;
    const Dimension hole_dims = node->dims_;
    features_[hole_name] = make_pair(hole_type, hole_dims);
  }
  return make_shared<Feature>(*node);
}

ast_ptr MapHoles::Visit(Num* node) { return make_shared<Num>(*node); }

ast_ptr MapHoles::Visit(Param* node) {
  const string& hole_name = node->name_;
  if (reset_params_) {
    node->current_value_ = nullptr;
  }
  if (srtrize_ && node->current_value_ != nullptr) {
    Param srtr_param(node->name_ + "A", node->dims_, node->type_);
    BinOp mod(make_shared<Param>(*node), make_shared<Param>(srtr_param), "Plus");
    return make_shared<BinOp>(mod);
  }
  parameters_.insert(hole_name);
  return make_shared<Param>(*node);
}

ast_ptr MapHoles::Visit(UnOp* node) {
  ast_ptr input = node->input_->Accept(this);
  depth_ += 1;
  if (srtrize_) {
    UnOp srtrd(input, node->op_);
    node = &srtrd;
    return make_shared<UnOp>(srtrd);
  }
  return make_shared<UnOp>(*node);
}

ast_ptr MapHoles::Visit(Var* node) { return make_shared<Var>(*node); }

ast_ptr MapHoles::Visit(Vec* node) { return make_shared<Vec>(*node); }

unordered_map<string, pair<Type, Dimension>> MapHoles::GetFeatureHoles() const {
  return features_;
}

unordered_set<string> MapHoles::GetParameterHoles() const {
  return parameters_;
}

bool MapHoles::IsRelative() const {
  return (depth_ > 1 || is_relative_);
}

void MapHoles::Reset() {
  features_.clear();
  parameters_.clear();
}

FillHole::FillHole(const string& target_name, const ast_ptr& new_value)
    : target_name_(target_name), new_value_(new_value) {}

ast_ptr FillHole::Visit(AST* node) { return ast_ptr(node); }

ast_ptr FillHole::Visit(BinOp* node) {
  node->left_->Accept(this);
  node->right_->Accept(this);
  return make_shared<BinOp>(*node);
}

ast_ptr FillHole::Visit(Bool* node) { return make_shared<Bool>(*node); }

ast_ptr FillHole::Visit(Feature* node) {
  if (node->name_ == target_name_ && node->current_value_ == nullptr) {
    node->current_value_ = new_value_;
  }
  if (node->current_value_ != nullptr) {
    node->current_value_->Accept(this);
  }
  return make_shared<Feature>(*node);
}

ast_ptr FillHole::Visit(Num* node) { return make_shared<Num>(*node); }

ast_ptr FillHole::Visit(Param* node) {
  if (node->name_ == target_name_ && node->current_value_ != nullptr) {
    // node->current_value_ = Plus(node->current_value_, new_value_);
  } else if (node->name_ == target_name_) {
    node->current_value_ = new_value_;
  }
  if (node->current_value_ != nullptr) {
    node->current_value_->Accept(this);
  }
  return make_shared<Param>(*node);
}

ast_ptr FillHole::Visit(UnOp* node) {
  node->input_->Accept(this);
  return make_shared<UnOp>(*node);
}

ast_ptr FillHole::Visit(Var* node) { return make_shared<Var>(*node); }

ast_ptr FillHole::Visit(Vec* node) { return make_shared<Vec>(*node); }

}  // namespace AST
