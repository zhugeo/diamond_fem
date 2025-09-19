#ifndef _DIAMOND_FEM_GEOMETRY_CORE_HPP
#define _DIAMOND_FEM_GEOMETRY_CORE_HPP

#include <vector>

#include <geometry/geometry_fwd.hpp>

namespace diamond_fem::geometry {

constexpr double EPSILON = 1e-5;

struct BoundingBox {
  double x_min, x_max, y_min, y_max;
};

BoundingBox BoundingBoxFromPoints(const std::vector<Point> &points);

BoundingBox CombineBoundingBoxes(const std::vector<BoundingBox> &boxes);

bool IsNear(double a, double b);

bool IsInRange(double x, double a, double b);

bool PointsNear(const Point &p1, const Point &p2);

} // namespace diamond_fem::geometry

#endif // _DIAMOND_FEM_GEOMETRY_CORE_HPP
