#include "ast.hpp"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <exception>
#include <iostream>
#include <ostream>
#include <stdexcept>

#include "library_functions.hpp"
#include "util/timer.h"

using namespace std;
using Eigen::Vector2f;
using Eigen::Vector3i;
using nlohmann::json;

namespace AST {

ValueProxy::ValueProxy(SymEntry const* owner) : owner_(owner) {}

ValueProxy::operator float() const { return owner_->GetFloat(); }

ValueProxy::operator Vector2f() const { return owner_->GetVector(); }

ValueProxy::operator string() const { return owner_->GetString(); }

SymEntry::SymEntry() : float_value_(0), type_(NUM) {}

SymEntry::SymEntry(const bool value) : bool_value_(value), type_(BOOL) {}

SymEntry::SymEntry(const float value) : float_value_(value), type_(NUM) {}

SymEntry::SymEntry(const string value) : string_value_(value), type_(STATE) {}

SymEntry::SymEntry(const Eigen::Vector2f& value)
    : vec_value_(value), type_(VEC) {}

ValueProxy SymEntry::GetValue() { return ValueProxy(this); }

bool SymEntry::GetBool() const {
  if (type_ != BOOL) {
    throw invalid_argument(
        "Attempted to get Boolean "
        "value from non-Boolean symbol.");
  }
  return bool_value_;
}

float SymEntry::GetFloat() const {
  if (type_ != NUM) {
    throw invalid_argument(
        "Attempted to get float "
        "value from non-float symbol.");
  }
  return float_value_;
}

string SymEntry::GetString() const {
  if (type_ != STATE) {
    throw invalid_argument(
        "Attempted to get string "
        "value from non-state symbol.");
  }
  return string_value_;
}

Vector2f SymEntry::GetVector() const {
  if (type_ != VEC) {
    throw invalid_argument(
        "Attempted to get vector "
        "value from non-vector symbol.");
  }
  return vec_value_;
}

Type SymEntry::GetType() const { return type_; }

bool SymEntry::operator==(const SymEntry& other) const {
  if (type_ != other.type_) {
    return false;
  }

  switch (type_) {
    case BOOL:
      return bool_value_ == other.bool_value_;
    case NUM:
      return float_value_ == other.float_value_;
    case VEC:
      return vec_value_ == other.vec_value_;
    case STATE:
      return string_value_ == other.string_value_;
    default:
      throw invalid_argument("Invalid SymEntry");
  }
}

ostream& operator<<(ostream& stream, const SymEntry& symentry) {
  switch (symentry.type_) {
    case BOOL:
      stream << symentry.GetBool();
      break;
    case NUM:
      stream << symentry.GetFloat();
      break;
    case VEC:
      stream << symentry.GetVector();
      break;
    default:
      throw invalid_argument("Invalid SymEntry");
  }
  return stream;
}

bool Example::operator==(const Example& other) const {
  return result_ == other.result_ && symbol_table_ == other.symbol_table_;
}

std::ostream& operator<<(std::ostream& stream, const Example& example) {
  stream << "in:";
  for (auto& name_value : example.symbol_table_) {
    const string& name = name_value.first;
    const SymEntry& value = name_value.second;
    stream << "\t" << name << "=" << value;
  }
  stream << ",\tout:\t" << example.result_;
  return stream;
}

// Constructors
AST::AST(const Dimension& dims, const Type& type) :
  dims_(dims), type_(type), priority(-1), symbolic_(false) {
}

AST::AST(const Dimension& dims, const Type& type, const bool& symbolic) :
  dims_(dims), type_(type), priority(-1), symbolic_(symbolic) {
}

AST::~AST(){};

TernOp::TernOp(ast_ptr x, ast_ptr a, ast_ptr b, const string& op)
    : AST({0, 0, 0}, OP), x_(x), a_(a), b_(b), op_(op) {}

TernOp::TernOp(ast_ptr x, ast_ptr a, ast_ptr b, const string& op, const Type& type,
             const Dimension& dim)
    : AST(dim, type), x_(x), a_(a), b_(b), op_(op) {}

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op)
    : AST({0, 0, 0}, OP), left_(left), right_(right), op_(op) {}

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op, const Type& type,
             const Dimension& dim)
    : AST(dim, type), left_(left), right_(right), op_(op) {}

UnOp::UnOp(ast_ptr input, const string& op)
    : AST({0, 0, 0}, OP), input_(input), op_(op) {}

UnOp::UnOp(ast_ptr input, const string& op, const Type& type,
           const Dimension& dim)
    : AST(dim, type), input_(input), op_(op) {}

Feature::Feature(const string& name, const Dimension& dims, const Type& type)
    : AST(dims, type), name_(name) {}

Var::Var(const string& name, const Dimension& dims, const Type& type)
    : AST(dims, type), name_(name) {}

Param::Param(const string& name, const Dimension& dims, const Type& type)
    : AST(dims, type, true), name_(name) {}

Vec::Vec(Vector2f value, Vector3i dims) : AST(dims, VEC), value_(value) {}

Bool::Bool(const bool& value) : AST({0, 0, 0}, BOOL), value_(value) {}

Num::Num(const float& value, const Dimension& dims)
    : AST(dims, NUM), value_(value) {}

// Necessary to get the automatic casting correct
ast_ptr AST::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr TernOp::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr BinOp::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr UnOp::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Bool::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Feature::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Num::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Param::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Var::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Vec::Accept(class Visitor* v) { return v->Visit(this); }
// End Casting Calls

// To and From Json Calls
//
ast_ptr AstFromJson(const json& input) {
  const string node_type = input["node"];
  Vec temp({0.0, 0.0}, {0, 0, 0});
  ast_ptr temp_ptr = make_shared<Vec>(temp);
  if (node_type == "BinOp") {
    BinOp node(temp_ptr, temp_ptr, "Plus");
    return node.FromJson(input);
  } else if (node_type == "UnOp") {
    UnOp node(temp_ptr, "Norm");
    return node.FromJson(input);
  } else if (node_type == "Feature") {
    Feature node("null", {0, 0, 0}, NODE);
    return node.FromJson(input);
  } else if (node_type == "Var") {
    Var node("null", {0, 0, 0}, NODE);
    return node.FromJson(input);
  } else if (node_type == "Param") {
    Param node("null", {0, 0, 0}, NODE);
    return node.FromJson(input);
  } else if (node_type == "Vec") {
    return temp.FromJson(input);
  } else if (node_type == "Bool") {
    Bool node(false);
    return node.FromJson(input);
  } else if (node_type == "Num") {
    Num node(0.0, {0, 0, 0});
    return node.FromJson(input);
  }
  cout << "ERROR Loading from Json: Unrecognized Node Type" << endl;
  return temp_ptr;
}

json TernOp::ToJson() {
  json output;
  output["node"] = "TernOp";
  output["type"] = type_;
  output["op"] = op_;
  output["symbolic"] = symbolic_;
  output["dim"] = {dims_.x(), dims_.y(), dims_.z()};
  output["x"] = x_->ToJson();
  output["a"] = a_->ToJson();
  output["b"] = b_->ToJson();

  return output;
}

ast_ptr TernOp::FromJson(const json& input) {
  ast_ptr x = AstFromJson(input["x"]);
  ast_ptr a = AstFromJson(input["a"]);
  ast_ptr b = AstFromJson(input["b"]);
  const string op = input["op"];
  const Type type = input["type"];
  const vector<int> dims = input["dim"];
  const Dimension dim({dims[0], dims[1], dims[2]});
  TernOp output(x, a, b, op, type, dim);
  return make_shared<TernOp>(output);
}

json BinOp::ToJson() {
  json output;
  output["node"] = "BinOp";
  output["type"] = type_;
  output["op"] = op_;
  output["symbolic"] = symbolic_;
  output["dim"] = {dims_.x(), dims_.y(), dims_.z()};
  output["left"] = left_->ToJson();
  output["right"] = right_->ToJson();

  return output;
}

ast_ptr BinOp::FromJson(const json& input) {
  ast_ptr left = AstFromJson(input["left"]);
  ast_ptr right = AstFromJson(input["right"]);
  const string op = input["op"];
  const Type type = input["type"];
  const vector<int> dims = input["dim"];
  const Dimension dim({dims[0], dims[1], dims[2]});
  BinOp output(left, right, op, type, dim);
  return make_shared<BinOp>(output);
}

json UnOp::ToJson() {
  json output;
  output["node"] = "UnOp";
  output["type"] = type_;
  output["op"] = op_;
  output["symbolic"] = symbolic_;
  output["dim"] = {dims_.x(), dims_.y(), dims_.z()};
  output["input"] = input_->ToJson();

  return output;
}

ast_ptr UnOp::FromJson(const json& input) {
  ast_ptr left = AstFromJson(input["input"]);
  const string op = input["op"];
  const Type type = input["type"];
  const vector<int> dims = input["dim"];
  const Dimension dim({dims[0], dims[1], dims[2]});
  UnOp output(left, op, type, dim);
  return make_shared<UnOp>(output);
}

json Feature::ToJson() {
  json output;
  output["node"] = "Feature";
  output["type"] = type_;
  output["name"] = name_;
  output["symbolic"] = symbolic_;
  output["dim"] = {dims_.x(), dims_.y(), dims_.z()};
  if (current_value_ == nullptr) {
    output["value"] = "null";
  } else {
    output["value"] = current_value_->ToJson();
  }
  return output;
}

ast_ptr Feature::FromJson(const json& input) {
  const string name = input["name"];
  const Type type = input["type"];
  const vector<int> dims = input["dim"];
  const Dimension dim({dims[0], dims[1], dims[2]});
  Feature output(name, dim, type);
  if (input["value"] != "null") {
    output.current_value_ = AstFromJson(input["value"]);
  }
  return make_shared<Feature>(output);
}

json Var::ToJson() {
  json output;
  output["node"] = "Var";
  output["type"] = type_;
  output["name"] = name_;
  output["symbolic"] = symbolic_;
  output["dim"] = {dims_.x(), dims_.y(), dims_.z()};
  return output;
}

ast_ptr Var::FromJson(const json& input) {
  const string name = input["name"];
  const Type type = input["type"];
  const vector<int> dims = input["dim"];
  const Dimension dim({dims[0], dims[1], dims[2]});
  Var output(name, dim, type);
  return make_shared<Var>(output);
}

json Param::ToJson() {
  json output;
  output["node"] = "Param";
  output["type"] = type_;
  output["name"] = name_;
  output["symbolic"] = symbolic_;
  output["dim"] = {dims_.x(), dims_.y(), dims_.z()};
  if (current_value_ == nullptr) {
    output["value"] = "null";
  } else {
    output["value"] = current_value_->ToJson();
  }
  return output;
}

ast_ptr Param::FromJson(const json& input) {
  const string name = input["name"];
  const Type type = input["type"];
  const vector<int> dims = input["dim"];
  const Dimension dim({dims[0], dims[1], dims[2]});
  Param output(name, dim, type);
  if (input["value"] != "null") {
    output.current_value_ = AstFromJson(input["value"]);
  }
  return make_shared<Param>(output);
}

json Vec::ToJson() {
  json output;
  output["node"] = "Vec";
  output["type"] = type_;
  output["value"] = {value_.x(), value_.y()};
  output["symbolic"] = symbolic_;
  output["dim"] = {dims_.x(), dims_.y(), dims_.z()};
  return output;
}

ast_ptr Vec::FromJson(const json& input) {
  const vector<int> dims = input["dim"];
  const Dimension dim({dims[0], dims[1], dims[2]});
  const vector<float> vect = input["value"];
  const Vector2f value(vect[0], vect[1]);
  Vec output(value, dim);
  return make_shared<Vec>(output);
}

json Bool::ToJson() {
  json output;
  output["node"] = "Bool";
  output["type"] = type_;
  output["value"] = value_;
  output["symbolic"] = symbolic_;
  output["dim"] = {dims_.x(), dims_.y(), dims_.z()};
  return output;
}

ast_ptr Bool::FromJson(const json& input) {
  const bool value(input["value"]);
  Bool output(value);
  return make_shared<Bool>(output);
}

json Num::ToJson() {
  json output;
  output["node"] = "Num";
  output["type"] = type_;
  output["value"] = value_;
  output["symbolic"] = symbolic_;
  output["dim"] = {dims_.x(), dims_.y(), dims_.z()};
  return output;
}

ast_ptr Num::FromJson(const json& input) {
  const vector<int> dims = input["dim"];
  const Dimension dim({dims[0], dims[1], dims[2]});
  const float value(input["value"]);
  Num output(value, dim);
  return make_shared<Num>(output);
}
// ToJson Calls


bool Var::operator==(const Var& other) const {
  return type_ == other.type_ && dims_ == other.dims_ && name_ == other.name_;
}

}  // namespace AST
