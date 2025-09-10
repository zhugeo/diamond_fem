#ifndef _DIAMOND_FEM_GEOMETRY_LINE_HPP
#define _DIAMOND_FEM_GEOMETRY_LINE_HPP

#include <geometry/curve.hpp>
#include <vector>

namespace diamond_fem::geometry {

class Line : public Curve {
private:
  Point p1_;
  Point p2_;

public:
  Line(const Point &p1, const Point &p2);

  virtual double Length() const override;
  /**
   * @brief Get point with natural parametrization
   */
  virtual Point GetParametricPoint(const double &t) const override;
  virtual std::vector<Point>
  GetIntersectionsWithAxis(const Point &point_on_axis,
                           const Vec &axis_direction) const override;
  virtual double DistanceToPoint(const Point &point) const override;
  virtual Vec NormalAtPoint(const Point &point) const override;
  virtual std::string Description() const override;
};

} // namespace diamond_fem::geometry

#endif // _DIAMOND_FEM_GEOMETRY_LINE_HPP
