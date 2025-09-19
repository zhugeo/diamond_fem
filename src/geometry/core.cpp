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

BoundingBox CombineBoundingBoxes(const std::vector<BoundingBox> &boxes) {
  auto points = std::vector<Point>();
  points.reserve(boxes.size() * 2);

  for (const auto &box : boxes) {
    points.push_back(Point(box.x_min, box.y_min));
    points.push_back(Point(box.x_max, box.y_max));
  }

  return BoundingBoxFromPoints(points);
}

bool IsNear(double a, double b) { return std::abs(a - b) < EPSILON; }

bool IsInRange(double x, double a, double b) {
  return x >= std::min(a, b) && x <= std::max(a, b);
}

bool PointsNear(const Point &p1, const Point &p2) {
  return std::abs(p1.GetX() - p2.GetX()) < EPSILON &&
         std::abs(p1.GetY() - p2.GetY()) < EPSILON;
}

} // namespace diamond_fem::geometry
