#ifndef _DIAMOND_FEM_GEOMETRY_ARC_HPP
#define _DIAMOND_FEM_GEOMETRY_ARC_HPP

#include <geometry/curve.hpp>

namespace diamond_fem::geometry {

class Arc : public Curve {
public:
  Arc(const Point &center, const Vec &radius_vector,
      const double &angle_counterclockwise);

  virtual double Length() const override;
  virtual Point GetParametricPoint(const double &t) const override;
  virtual std::vector<Point>
  GetIntersectionsWithAxis(const Point &point_on_axis,
                           const Vec &axis_direction) const override;
  virtual double DistanceToPoint(const Point &point) const override;
  virtual Vec NormalAtPoint(const Point &point) const override;
  virtual std::string Description() const override;

private:
  bool IsPointOnArc_(const Point &p) const;
  Point GetClosestPointOnArc_(const Point &p) const;

  /**
   * Start point of arc is center_ + radius_vector_.
   * Radius of arc is radius_vector_.Length()
   */
  Vec radius_vector_;
  Point center_;
  double angle_; // counterclockwise, radians
};

} // namespace diamond_fem::geometry

#endif // _DIAMOND_FEM_GEOMETRY_ARC_HPP
