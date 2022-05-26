#include <gtest/gtest.h>

#include <vector>

#include "nd_bool_array.hpp"

using std::vector;

TEST(ndarray, Construction) {
  vector<size_t> dims = {1, 2, 3};
  nd_bool_array test(dims);
  ASSERT_EQ(test.size(), 1 * 2 * 3);
  test.set({0, 1, 2}, true);
  ASSERT_EQ(test.get({0, 1, 2}), true);
}