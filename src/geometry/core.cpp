#include <geometry/core.hpp>

#include <cmath>
#include <algorithm>

namespace diamond_fem::geometry {

bool IsNear(double a, double b) { return std::abs(a - b) < EPSILON; }

bool IsInRange(double x, double a, double b) {
  return x >= std::min(a, b) && x <= std::max(a, b);
}

} // namespace diamond_fem::geometry
