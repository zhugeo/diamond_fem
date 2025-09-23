#include <geometry/arc.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <fmt/format.h>

#include <geometry/core.hpp>
#include <geometry/point.hpp>
#include <geometry/vec.hpp>

namespace diamond_fem::geometry {

namespace {

std::vector<Point> CircleAndLineIntersections(const Point &center,
                                              const double &radius,
                                              const Point &point_on_axis,
                                              const Vec &axis_direction) {
  const auto to_center = center - point_on_axis;

  // Project to_center onto line direction
  const auto dir_dot_dir = axis_direction.DotProduct(axis_direction);
  const auto dir_dot_to_center = axis_direction.DotProduct(to_center);

  // Closest point on line to circle center
  const auto t_closest = dir_dot_to_center / dir_dot_dir;
  const auto closest_point = point_on_axis + axis_direction * t_closest;

  const auto distance_to_center = (closest_point - center).Length();

  if (distance_to_center > radius) {
    return {};
  }

  if (IsNear(distance_to_center, radius)) {
    return {closest_point};
  }

  const auto half_chord_length =
      std::sqrt(radius * radius - distance_to_center * distance_to_center);
  const auto t_offset = half_chord_length / std::sqrt(dir_dot_dir);

  const auto intersection1 =
      point_on_axis + axis_direction * (t_closest - t_offset);
  const auto intersection2 =
      point_on_axis + axis_direction * (t_closest + t_offset);

  return {intersection1, intersection2};
}

} // namespace

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
  return RetainArcPoints_(CircleAndLineIntersections(
      center_, radius_vector_.Length(), point_on_axis, axis_direction));
}

double Arc::DistanceToPoint(const Point &point) const {
  return (GetClosestPointOnArc_(point) - point).Length();
}

Vec Arc::NormalAtPoint(const Point &point) const {
  Vec to_point = point - center_;
  return to_point.Normalized();
}

BoundingBox Arc::GetBoundingBox() const {
  auto candidates = std::vector<Point>();
  const auto radius = radius_vector_.Length();

  candidates.push_back(center_ + Vec(0, radius));
  candidates.push_back(center_ - Vec(0, radius));
  candidates.push_back(center_ + Vec(radius, 0));
  candidates.push_back(center_ - Vec(radius, 0));

  candidates.push_back(GetParametricPoint(0));
  candidates.push_back(GetParametricPoint(Length()));

  const auto filtered_candidates = RetainArcPoints_(std::move(candidates));

  return BoundingBoxFromPoints(filtered_candidates);
}

std::string Arc::Description() const {
  return fmt::format("Arc(Point({}, {}), Vec({}, {}), {})", center_.GetX(),
                     center_.GetY(), radius_vector_.GetX(),
                     radius_vector_.GetY(), angle_);
}

double Arc::GetPointParameter(const Point &point) const {
  const auto v = point - center_;

  const auto global_angle_of_point =
      std::atan2(radius_vector_.GetY(), radius_vector_.GetX());
  const auto zero_angle = std::atan2(v.GetY(), v.GetX());

  const auto local_angle_of_point =
      std::fmod(zero_angle - global_angle_of_point + 2 * M_PI, 2 * M_PI);

  return radius_vector_.Length() * local_angle_of_point;
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

std::vector<Point> Arc::RetainArcPoints_(std::vector<Point> points) const {
  auto retained_points = std::vector<Point>();

  for (const auto &point : points) {
    if (IsPointOnArc_(point)) {
      retained_points.push_back(point);
    }
  }

  return retained_points;
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
