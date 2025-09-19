#ifndef _DIAMOND_FEM_GEOMETRY_INTERVAL_HPP_
#define _DIAMOND_FEM_GEOMETRY_INTERVAL_HPP_

#include <vector>

#include <geometry/geometry_fwd.hpp>
#include <geometry/point.hpp>
#include <geometry/vec.hpp>

namespace diamond_fem::geometry{

class Interval {
public:
  Interval(const Point &start_point,
           const Point &end_point,
           const geometry::Point &point_on_grid_node,
           const geometry::Vec &direction_step);

  std::vector<Point> DoStepping() const;

private:
  Point start_point_;
  Point end_point_;
  geometry::Point point_on_grid_node_;
  geometry::Vec direction_step_;
};

}//namespace diamond_fem::geometry

#endif
