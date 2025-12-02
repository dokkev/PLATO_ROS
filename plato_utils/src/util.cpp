#include "plato_utils/util.hpp"

namespace util {

std::vector<double> ReorderJointState(const std::vector<double>& input,
                                      const std::vector<size_t>& index_map) {
  std::vector<double> output(index_map.size(), 0.0);
  for (size_t i = 0; i < index_map.size(); ++i) {
    const size_t src = index_map[i];
    if (src < input.size()) {
      output[i] = input[src];
    }
  }
  return output;
}

std::vector<double> ReorderToStandardJointOrder(const std::vector<double>& input) {
  static const std::array<size_t, 8> kMap = {4, 0, 1, 5, 7, 2, 6, 3};
  return ReorderJointState(input, std::vector<size_t>(kMap.begin(), kMap.end()));
}

}  // namespace util
