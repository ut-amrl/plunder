#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>

#include "library_functions.hpp"

using Eigen::Vector2f;

TEST(LibraryFunctions, PointInPolygonSimpleSquare) {
  const Polygon square = {{{1, 1}, {1, -1}, {-1, -1}, {-1, 1}}};
  const Vector2f point = {0, 0};
  ASSERT_TRUE(PointInPolygon(point, square));
}

TEST(LibraryFunctions, PointInPolygonPointOnPolygonEdge) {
  const Polygon square = {{{1, 1}, {1, -1}, {-1, -1}, {-1, 1}}};
  const Vector2f point = {1, 0};
  ASSERT_TRUE(PointInPolygon(point, square));
}

TEST(LibraryFunctions, PointInPolygonNotInSquare) {
  const Polygon square = {{{1, 1}, {1, -1}, {-1, -1}, {-1, 1}}};
  const Vector2f point = {2, 2};
  ASSERT_FALSE(PointInPolygon(point, square));
}

TEST(LibraryFunctions, ConvexHullTwoSquares) {
  const Polygon square1 = {{{0, 0}, {0, 1}, {1, 1}, {1, 0}}};
  const Polygon square2 = {{{2, 2}, {2, 3}, {3, 3}, {3, 2}}};
  const Polygon convex_hull = ConvexHull(square1, square2);
  ASSERT_EQ(6, convex_hull.vertices.size());
  ASSERT_EQ(Vector2f(0, 0), convex_hull.vertices[0]);
  ASSERT_EQ(Vector2f(1, 0), convex_hull.vertices[1]);
  ASSERT_EQ(Vector2f(3, 2), convex_hull.vertices[2]);
  ASSERT_EQ(Vector2f(3, 3), convex_hull.vertices[3]);
  ASSERT_EQ(Vector2f(2, 3), convex_hull.vertices[4]);
  ASSERT_EQ(Vector2f(0, 1), convex_hull.vertices[5]);
}