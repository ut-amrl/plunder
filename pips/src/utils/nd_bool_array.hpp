// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#include <vector>

class nd_bool_array {
  const std::vector<size_t> dims_;
  std::vector<bool> data_;

 public:
  nd_bool_array(const std::vector<size_t>& dims);
  bool get(const std::vector<size_t>& coords) const;
  void set(const std::vector<size_t>& coords, bool new_value);
  size_t size() const;

 private:
  size_t coords_to_index(const std::vector<size_t>& coords) const;
};
