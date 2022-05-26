// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <eigen3/Eigen/Core>
#include <vector>

#include "ast.hpp"

struct LineSegment {
  const Eigen::Vector2f a;
  const Eigen::Vector2f b;
};

struct Polygon {
  const std::vector<Eigen::Vector2f> vertices;
};

struct Ray {
  const Eigen::Vector2f origin;
  const Eigen::Vector2f direction;
};

AST::ast_ptr Plus(AST::ast_ptr left, AST::ast_ptr right);
AST::ast_ptr Minus(AST::ast_ptr left, AST::ast_ptr right);
AST::ast_ptr Times(AST::ast_ptr left, AST::ast_ptr right);
AST::ast_ptr DividedBy(AST::ast_ptr left, AST::ast_ptr right);
AST::ast_ptr AngleDist(AST::ast_ptr left, AST::ast_ptr right);

AST::ast_ptr Abs(AST::ast_ptr operand);
AST::ast_ptr Pow(AST::ast_ptr base, AST::ast_ptr power);
AST::ast_ptr Sq(AST::ast_ptr x);

AST::ast_ptr Cos(AST::ast_ptr theta);
AST::ast_ptr Sin(AST::ast_ptr theta);

AST::ast_ptr Cross(AST::ast_ptr u, AST::ast_ptr v);
AST::ast_ptr Dot(AST::ast_ptr u, AST::ast_ptr v);
AST::ast_ptr SqDist(AST::ast_ptr u, AST::ast_ptr v);
AST::ast_ptr Heading(AST::ast_ptr theta);
AST::ast_ptr Angle(AST::ast_ptr v);
AST::ast_ptr NormSq(AST::ast_ptr v);
AST::ast_ptr Perp(AST::ast_ptr v);
AST::ast_ptr VecX(AST::ast_ptr v);
AST::ast_ptr VecY(AST::ast_ptr v);

AST::ast_ptr And(AST::ast_ptr P, AST::ast_ptr Q);
AST::ast_ptr Or(AST::ast_ptr P, AST::ast_ptr Q);
AST::ast_ptr Not(AST::ast_ptr P);
AST::ast_ptr Eq(AST::ast_ptr x, AST::ast_ptr y);
AST::ast_ptr Lt(AST::ast_ptr x, AST::ast_ptr y);
AST::ast_ptr Gt(AST::ast_ptr x, AST::ast_ptr y);
AST::ast_ptr Lte(AST::ast_ptr x, AST::ast_ptr y);
AST::ast_ptr Gte(AST::ast_ptr x, AST::ast_ptr y);

// Tentative idea, function that loads presets from the examples
AST::ast_ptr StraightFreePathLength(
    AST::ast_ptr v,
    const std::vector<Eigen::Vector2f> obstacles);

float Average(std::vector<float> xs);
Polygon ConvexHull(Polygon a, Polygon b);
bool PointInPolygon(Eigen::Vector2f point, Polygon polygon);
bool RayIntersection(Ray ray, LineSegment line_segment);
