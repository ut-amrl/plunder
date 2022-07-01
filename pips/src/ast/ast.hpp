// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#include <eigen3/Eigen/Core>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <ostream>
#include <string>
#include <unordered_set>
#include <vector>

#ifndef SRC_AST_HPP_
#define SRC_AST_HPP_

namespace AST {

enum Type { NODE, VAR, NUM, VEC, OP, BOOL, STATE };

// Forward Declaration
class SymEntry;

// Used for comparing values without explicitly checking them.
class ValueProxy {
 public:
  ValueProxy(SymEntry const* owner);
  operator bool() const;
  operator float() const;
  operator Eigen::Vector2f() const;
  operator std::string() const;

 private:
  SymEntry const* owner_;
};

class SymEntry {
 public:
  SymEntry();
  SymEntry(const bool value);
  SymEntry(const std::string value);
  SymEntry(const float value);
  SymEntry(const Eigen::Vector2f& value);
  ValueProxy GetValue();
  bool GetBool() const;
  float GetFloat() const;
  std::string GetString() const;
  Eigen::Vector2f GetVector() const;
  Type GetType() const;
  bool operator==(const SymEntry& other) const;
  friend std::ostream& operator<<(std::ostream& stream,
                                  const SymEntry& symentry);
  std::string name_;

 private:
  bool bool_value_;
  float float_value_;
  std::string string_value_;
  Eigen::Vector2f vec_value_;
  Type type_;
};

std::ostream& operator<<(std::ostream& stream, const SymEntry& symentry);

// Format of input to all ASTs
struct Example {
  std::map<std::string, SymEntry> symbol_table_;
  std::vector<Eigen::Vector2f> obstacles_;
  SymEntry result_; // Output may not necessarily be a state
  SymEntry start_; // Start should be a state, but leaving as SymEntry
  bool operator==(const Example& other) const;
};

std::ostream& operator<<(std::ostream& stream, const Example& example);

// useful typedef for tracking purpose
typedef Eigen::Vector3i Dimension;

class AST {
 public:
  AST(const Dimension& dims, const Type& type);
  AST(const Dimension& dims, const Type& type, const bool& symbolic);
  virtual std::shared_ptr<AST> Accept(class Visitor* v) = 0;
  virtual nlohmann::json ToJson() = 0;
  virtual std::shared_ptr<AST> FromJson(const nlohmann::json&) = 0;
  virtual ~AST() = 0;
  const Dimension dims_;
  int priority = -1;
  const Type type_;
  bool symbolic_;

 private:
};
typedef std::shared_ptr<AST> ast_ptr;

class BinOp : public AST {
 public:
  BinOp(ast_ptr left, ast_ptr right, const std::string& op);
  BinOp(ast_ptr left, ast_ptr right, const std::string& op, const Type& type,
        const Dimension& dim);
  ast_ptr Accept(class Visitor* v);
  nlohmann::json ToJson();
  ast_ptr FromJson(const nlohmann::json&);
  ast_ptr left_;
  ast_ptr right_;
  const std::string op_;
  Type type_ = OP;

 private:
};
typedef std::shared_ptr<BinOp> bin_ptr;

class Bool : public AST {
 public:
  Bool(const bool& value);
  ast_ptr Accept(class Visitor* v);
  nlohmann::json ToJson();
  ast_ptr FromJson(const nlohmann::json&);
  bool value_;

 private:
};
typedef std::shared_ptr<Bool> bool_ptr;

class Feature : public AST {
 public:
  Feature(const std::string& name, const Dimension& dims, const Type& type);
  ast_ptr Accept(class Visitor* v);
  nlohmann::json ToJson();
  ast_ptr FromJson(const nlohmann::json&);
  const std::string name_;
  ast_ptr current_value_ = nullptr;

 private:
};
typedef std::shared_ptr<Feature> feature_ptr;

class Num : public AST {
 public:
  Num(const float& value, const Dimension& dims);
  ast_ptr Accept(class Visitor* v);
  nlohmann::json ToJson();
  ast_ptr FromJson(const nlohmann::json&);
  float value_;

 private:
};
typedef std::shared_ptr<Num> num_ptr;

class Param : public AST {
 public:
  Param(const std::string& name, const Dimension& dims, const Type& type);
  ast_ptr Accept(class Visitor* v);
  const std::string name_;
  nlohmann::json ToJson();
  ast_ptr FromJson(const nlohmann::json&);
  ast_ptr current_value_ = nullptr;

 private:
};
typedef std::shared_ptr<Param> param_ptr;

class UnOp : public AST {
 public:
  UnOp(ast_ptr input, const std::string& op);
  UnOp(ast_ptr input, const std::string& op, const Type& type,
       const Dimension& dim);
  ast_ptr Accept(class Visitor* v);
  nlohmann::json ToJson();
  ast_ptr FromJson(const nlohmann::json&);
  ast_ptr input_;
  const std::string op_;
  Type type_ = OP;

 private:
};
typedef std::shared_ptr<UnOp> un_ptr;

class Var : public AST {
 public:
  Var(const std::string& name, const Dimension& dims, const Type& type);
  ast_ptr Accept(class Visitor* v);
  nlohmann::json ToJson();
  ast_ptr FromJson(const nlohmann::json&);
  bool operator==(const Var& other) const;
  const std::string name_;

 private:
};
typedef std::shared_ptr<Var> var_ptr;

class Vec : public AST {
 public:
  Vec(Eigen::Vector2f value, Eigen::Vector3i dims);
  ast_ptr Accept(class Visitor* v);
  nlohmann::json ToJson();
  ast_ptr FromJson(const nlohmann::json&);
  Eigen::Vector2f value_;
  Type type_ = VEC;

 private:
};
typedef std::shared_ptr<Vec> vec_ptr;

class Visitor {
 public:
  virtual ast_ptr Visit(AST* node) = 0;
  virtual ast_ptr Visit(BinOp* node) = 0;
  virtual ast_ptr Visit(Bool* node) = 0;
  virtual ast_ptr Visit(Feature* node) = 0;
  virtual ast_ptr Visit(Num* node) = 0;
  virtual ast_ptr Visit(Param* node) = 0;
  virtual ast_ptr Visit(UnOp* node) = 0;
  virtual ast_ptr Visit(Var* node) = 0;
  virtual ast_ptr Visit(Vec* node) = 0;
};

ast_ptr AstFromJson(const nlohmann::json&);

}  // namespace AST

namespace std {

// TODO(simon) Check whether these hash functions are any good.

template <>
struct hash<AST::SymEntry> {
  size_t operator()(const AST::SymEntry& se) const {
    size_t hash = std::hash<std::string>()(se.name_);
    switch (se.GetType()) {
      case AST::BOOL:
        hash += std::hash<bool>()(se.GetBool());
        return hash;
      case AST::NUM:
        hash += std::hash<float>()(se.GetFloat());
        return hash;
      case AST::VEC:
        hash += std::hash<float>()(se.GetVector().norm());
        return hash;
      case AST::STATE:
        hash += std::hash<string>()(se.GetString());
        return hash;
      default:
        (void)0;
    }
    throw std::invalid_argument("unknown SymEntry type");
  }
};

template <>
struct hash<AST::Example> {
  size_t operator()(const AST::Example& ex) const {
    size_t hash = std::hash<AST::SymEntry>()(ex.result_);
    const auto& p = *(ex.symbol_table_.cbegin());
    hash += std::hash<std::string>()(p.first);
    hash += std::hash<AST::SymEntry>()(p.second);
    return hash;
  }
};

template <>
struct hash<AST::Var> {
  size_t operator()(const AST::Var& var) const {
    return std::hash<std::string>()(var.name_);
  }
};

}  // namespace std

#endif  // SRC_AST_HPP
