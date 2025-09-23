#ifndef _DIAMOND_FEM_GEOMETRY_POINT_HPP
#define _DIAMOND_FEM_GEOMETRY_POINT_HPP

#include <geometry/geometry_fwd.hpp>
#include <ostream>

namespace diamond_fem::geometry {

class Point {
public:
  explicit Point(double x, double y);

  double GetX() const;
  double GetY() const;

  bool operator==(const Point &other) const;
  bool operator!=(const Point &other) const;

  Vec operator-(const Point &other) const;

  Point operator+(const Vec &vec) const;
  Point operator-(const Vec &vec) const;

  double DistanceTo(const Point &other) const;

  friend std::ostream &operator<<(std::ostream &os, const Point &point);

private:
  double x_;
  double y_;
};

} // namespace diamond_fem::geometry

#endif // _DIAMOND_FEM_GEOMETRY_POINT_HPP
