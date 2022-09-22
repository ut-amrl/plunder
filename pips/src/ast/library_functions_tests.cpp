#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>

#include "library_functions.hpp"

using Eigen::Vector2f;

TEST(LibraryFunctions, FlipTest){
    Num p4(0.4, {0, 0, 0}); Num p5(0.5, {0, 0, 0}); Num p6(0.6, {0, 0, 0});
    ASSERT_EQ(0, Flip(make_shared<Num>(p4), make_shared<Bool>(false)));
    ASSERT_EQ(0, Flip(make_shared<Num>(p5), make_shared<Bool>(false)));
    ASSERT_EQ(1, Flip(make_shared<Num>(p6), make_shared<Bool>(false)));

    int yesCount = 0;
    int noCount = 0;
    for(int i = 0; i < 100; i++) {
        bool trial = dynamic_pointer_cast<Bool>(Flip(make_shared<Num>(p4), make_shared<Bool>(true)))->value_;
        cout << trial << endl;
        if(trial)
            yesCount++;
        else
            noCount++;
    }

    cout <<  "Yes count for p=" << 0.4 << ": " << yesCount << "; No count: " << noCount << endl;
}

TEST(LibraryFunctions, LogisticTest) {
  ast_ptr one = make_shared<Num>(Num(1, {0, 0, 0}));
  ast_ptr neg = make_shared<Num>(Num(-1, {0, 0, 0}));
  ast_ptr two = make_shared<Num>(Num(2, {0, 0, 0}));
  ast_ptr three = make_shared<Num>(Num(3, {0, 0, 0}));
  ast_ptr four = make_shared<Num>(Num(4, {0, 0, 0}));

  ASSERT_NEAR(0.018, Logistic(one, two, four), 0.001);
  ASSERT_NEAR(0.119, Logistic(one, three, one), 0.001);
  ASSERT_NEAR(0.5, Logistic(neg, neg, two), 0.001);
}

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

int main(int argc, char ∗∗argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}