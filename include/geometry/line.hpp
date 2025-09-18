#ifndef _DIAMOND_FEM_GEOMETRY_LINE_HPP
#define _DIAMOND_FEM_GEOMETRY_LINE_HPP

#include <vector>

#include <geometry/curve.hpp>
#include <geometry/point.hpp>

namespace diamond_fem::geometry {

class Line : public Curve {
public:
  Line(const Point &p1, const Point &p2);

  virtual double Length() const override;
  virtual Point GetParametricPoint(const double &t) const override;
  virtual std::vector<Point>
  GetIntersectionsWithAxis(const Point &point_on_axis,
                           const Vec &axis_direction) const override;
  virtual double DistanceToPoint(const Point &point) const override;
  virtual Vec NormalAtPoint(const Point &point) const override;
  virtual BoundingBox GetBoundingBox() const override;
  virtual std::string Description() const override;

private:
  Point p1_;
  Point p2_;
};

} // namespace diamond_fem::geometry

#endif // _DIAMOND_FEM_GEOMETRY_LINE_HPP
