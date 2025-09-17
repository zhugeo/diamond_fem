#include <geometry/arc.hpp>

#include <cmath>
#include <string>
#include <vector>

#include <fmt/format.h>

#include <geometry/core.hpp>
#include <geometry/point.hpp>
#include <geometry/vec.hpp>

namespace diamond_fem::geometry {

Arc::Arc(const Point &center, const Vec &radius_vector, const double &angle)
    : center_(center), radius_vector_(radius_vector), angle_(angle) {}

double Arc::Length() const {
  double radius = radius_vector_.Length();
  return radius * angle_;
}

Point Arc::GetParametricPoint(const double &t) const {
  if (angle_ == 0.0 || radius_vector_.Length() == 0.0) {
    return center_ + radius_vector_;
  }

  double radius = radius_vector_.Length();
  double current_angle = std::min(std::max(t / radius, 0.0), angle_);
  double start_angle = std::atan2(radius_vector_.GetY(), radius_vector_.GetX());
  double total_angle = start_angle + current_angle;

  double x = center_.GetX() + radius * std::cos(total_angle);
  double y = center_.GetY() + radius * std::sin(total_angle);

  return Point(x, y);
}

std::vector<Point>
Arc::GetIntersectionsWithAxis(const Point &point_on_axis,
                              const Vec &axis_direction) const {
  const auto radius = radius_vector_.Length();
  const auto to_center = center_ - point_on_axis;

  // Project to_center onto line direction
  const auto dir_dot_dir = axis_direction.DotProduct(axis_direction);
  const auto dir_dot_to_center = axis_direction.DotProduct(to_center);

  // Closest point on line to circle center
  const auto t_closest = dir_dot_to_center / dir_dot_dir;
  const auto closest_point = point_on_axis + axis_direction * t_closest;

  const auto distance_to_center = (closest_point - center_).Length();

  if (distance_to_center > radius) {
    return {};
  }

  const auto retain_arc_points = [&](const std::vector<Point> &points) {
    auto retained_points = std::vector<Point>();
    for (const auto &point : points) {
      if (IsPointOnArc_(point)) {
        retained_points.push_back(point);
      }
    }
    return retained_points;
  };

  if (IsNear(distance_to_center, radius)) {
    return retain_arc_points({closest_point});
  }

  const auto half_chord_length =
      std::sqrt(radius * radius - distance_to_center * distance_to_center);
  const auto t_offset = half_chord_length / std::sqrt(dir_dot_dir);

  const auto intersection1 =
      point_on_axis + axis_direction * (t_closest - t_offset);
  const auto intersection2 =
      point_on_axis + axis_direction * (t_closest + t_offset);

  return retain_arc_points({intersection1, intersection2});
}

double Arc::DistanceToPoint(const Point &point) const {
  return (GetClosestPointOnArc_(point) - point).Length();
}

Vec Arc::NormalAtPoint(const Point &point) const {
  Vec to_point = point - center_;
  return to_point.Normalized();
}

std::string Arc::Description() const {
  return fmt::format("Arc(Point({}, {}), Vec({}, {}), {})", center_.GetX(),
                     center_.GetY(), radius_vector_.GetX(),
                     radius_vector_.GetY(), angle_);
}

bool Arc::IsPointOnArc_(const Point &p) const {
  const auto radius = radius_vector_.Length();
  const auto v = p - center_;

  if (!IsNear(v.Length(), radius)) {
    return false;
  }

  const auto global_angle_of_p =
      std::atan2(radius_vector_.GetY(), radius_vector_.GetX());
  const auto zero_angle = std::atan2(v.GetY(), v.GetX());

  const auto local_angle_of_p =
      std::fmod(zero_angle - global_angle_of_p + 2 * M_PI, 2 * M_PI);

  return local_angle_of_p <= angle_ + EPSILON && local_angle_of_p >= -EPSILON;
}

Point Arc::GetClosestPointOnArc_(const Point &p) const {
  const auto v = p - center_;
  const auto v2 = v * (radius_vector_.Length() / v.Length());
  const auto proj = center_ + v2;

  if (IsPointOnArc_(proj)) {
    return proj;
  }

  const auto start = GetParametricPoint(0);
  const auto end = GetParametricPoint(Length());
  return (p - start).Length() < (p - end).Length() ? start : end;
}

} // namespace diamond_fem::geometry
