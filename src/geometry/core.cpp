#include <geometry/core.hpp>

#include <algorithm>
#include <cmath>

#include <geometry/point.hpp>

namespace diamond_fem::geometry {

BoundingBox BoundingBoxFromPoints(const std::vector<Point> &points) {
  auto bbox = BoundingBox{
      .x_min = std::numeric_limits<double>::max(),
      .x_max = std::numeric_limits<double>::min(),
      .y_min = std::numeric_limits<double>::max(),
      .y_max = std::numeric_limits<double>::min(),
  };

  for (const auto &point : points) {
    bbox.x_min = std::min(bbox.x_min, point.GetX());
    bbox.x_max = std::max(bbox.x_max, point.GetX());
    bbox.y_min = std::min(bbox.y_min, point.GetY());
    bbox.y_max = std::max(bbox.y_max, point.GetY());
  }

  return bbox;
}

BoundingBox CombineBoundingBoxes(const BoundingBox &a, const BoundingBox &b) {
  return {
      .x_min = std::min(a.x_min, b.x_min),
      .x_max = std::max(a.x_max, b.x_max),
      .y_min = std::min(a.y_min, b.y_min),
      .y_max = std::max(a.y_max, b.y_max),
  };
}

bool IsNear(double a, double b) { return std::abs(a - b) < EPSILON; }

bool IsInRange(double x, double a, double b) {
  return x >= std::min(a, b) && x <= std::max(a, b);
}

} // namespace diamond_fem::geometry
