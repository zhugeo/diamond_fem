#include "geometry/core.hpp"
#include <geometry/interval.hpp>
#include <geometry/line.hpp>

namespace diamond_fem::geometry {

Interval::Interval(const Point &start_point, const Point &end_point,
                   const geometry::Point &point_on_grid_node,
                   const geometry::Vec &direction_step)
    : start_point_(start_point), end_point_(end_point),
      point_on_grid_node_(point_on_grid_node), direction_step_(direction_step) {
}

std::vector<Point> Interval::DoStepping() const {
  const auto segment = geometry::Line(start_point_, end_point_);
  const auto step_length = direction_step_.Length();

  // If very short (or single point) segment, try return one point

  // if point on grid node is on segment, move it away
  auto point_on_grid = point_on_grid_node_;
  if (segment.DistanceToPoint(point_on_grid) == 0) {
    point_on_grid =
        point_on_grid +
        direction_step_ * std::ceil(segment.Length() / step_length + 1);
  }

  const auto d1 = (point_on_grid - start_point_).Length();
  const auto d2 = (point_on_grid - end_point_).Length();

  // p1 is the point nearest to the grid node, p2 is the most far
  const auto p1 = d1 < d2 ? start_point_ : end_point_;
  const auto p2 = d1 < d2 ? end_point_ : start_point_;

  // direction_step_ that oriented along stepping direction.
  // If there is single point segment, direction is kept as it is
  auto oriented_direction_step = direction_step_;
  if (!PointsNear(p1, p2)) {
    oriented_direction_step =
        (p2 - p1).Normalized().DotProduct(direction_step_.Normalized()) *
        direction_step_;
  }

  const auto first_point =
      point_on_grid +
      oriented_direction_step *
          std::ceil((p1 - point_on_grid).Length() / step_length);

  auto points_count = std::floor((p2 - first_point).Length() / step_length) + 1;

  if (segment.DistanceToPoint(first_point) > EPSILON) {
    return {};
  }

  auto result = std::vector<Point>();
  result.reserve(points_count);

  for (int i = 0; i < points_count; i++) {
    const auto point = first_point + i * oriented_direction_step;
    result.push_back(point);
  }

  return result;
}

} // namespace diamond_fem::geometry
