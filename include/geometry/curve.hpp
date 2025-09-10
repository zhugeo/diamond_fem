#ifndef _DIAMOND_FEM_GEOMETRY_CURVE_HPP
#define _DIAMOND_FEM_GEOMETRY_CURVE_HPP

#include <geometry/point.hpp>
#include <geometry/vec.hpp>
#include <vector>

namespace diamond_fem::geometry {

class Curve {
public:
  virtual double Length() const = 0;
  /**
   * @brief Get point with natural parametrization
   */
  virtual Point GetParametricPoint(const double &t) const = 0;
  virtual std::vector<Point>
  GetIntersectionsWithAxis(const Point &point_on_axis,
                           const Vec &axis_direction) const = 0;
  virtual double DistanceToPoint(const Point &point) const = 0;
  /**
   * @returns Normal, guaranteed to be normalized
   */
  virtual Vec NormalAtPoint(const Point &point) const = 0;
  virtual std::string Description() const = 0;
};

} // namespace diamond_fem::geometry

#endif // _DIAMOND_FEM_GEOMETRY_CURVE_HPP
