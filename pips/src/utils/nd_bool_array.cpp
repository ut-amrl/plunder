#include "nd_bool_array.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

using namespace std;

nd_bool_array::nd_bool_array(const vector<size_t>& dims) : dims_(dims) {
  if (dims_.empty()) {
    throw invalid_argument("nd_bool_array must have /some/ dimensions");
  }
  size_t elem_count = 1;
  for (const size_t& dim : dims) {
    if (dim == 0) {
      throw invalid_argument("nd_bool_array can't have zero size dimensions!");
    }
    elem_count *= dim;
  }
  data_.resize(elem_count);
}

bool nd_bool_array::get(const std::vector<size_t>& coords) const {
  size_t index = coords_to_index(coords);
  return data_[index];
}

void nd_bool_array::set(const std::vector<size_t>& coords, bool new_value) {
  size_t index = coords_to_index(coords);
  data_[index] = new_value;
}

size_t nd_bool_array::size() const { return data_.size(); }

size_t nd_bool_array::coords_to_index(const vector<size_t>& coords) const {
  size_t index = 0;
  for (size_t i = 0; i < dims_.size(); ++i) {
    if (coords[i] >= dims_[i]) {
      throw out_of_range("index out of bounds!");
    }
    index += pow<size_t>(dims_[i], dims_.size() - i - 1) * coords[i];
  }
  return index;
}