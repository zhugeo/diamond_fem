#include <cmath>
#include <geometry/arc.hpp>

#include <fmt/format.h>

#include <geometry/point.hpp>
#include <geometry/utils.hpp>
#include <geometry/vec.hpp>

namespace diamond_fem::geometry {

namespace {

bool IsPointOnArc(const Point &p, const Point &center, const Vec &radius_vector,
                  double angle, double radius) {
  const auto v = p - center;

  if (!IsNear(v.Length(), radius))
    return false;

  double angle1 = std::atan2(radius_vector.GetY(), radius_vector.GetX());
  double angle2 = std::atan2(v.GetY(), v.GetX());
  double diff = std::fmod(angle2 - angle1 + 2 * M_PI, 2 * M_PI);

  return diff <= angle + 1e-6;
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
  const auto normalized_axis = axis_direction.Normalized();

  const auto center_to_point = point_on_axis - center_;

  const auto a = normalized_axis.GetX() * normalized_axis.GetX() +
                 normalized_axis.GetY() * normalized_axis.GetY();

  const auto b = 2 * (normalized_axis.GetX() * (center_to_point.GetX()) +
                      normalized_axis.GetY() * (center_to_point.GetY()));

  const auto c = center_to_point.GetX() * center_to_point.GetX() +
                 center_to_point.GetY() * center_to_point.GetY() -
                 radius_vector_.Length() * radius_vector_.Length();

  const auto discriminant = std::pow(b, 2) - 4 * a * c;
  if (discriminant < 0) {
    return {};
  }

  auto solutions = std::vector<double>();
  if (discriminant == 0) {
    solutions.push_back(-b / (2 * a));
  } else {
    solutions.push_back((-b + std::sqrt(discriminant)) / (2 * a));
    solutions.push_back((-b - std::sqrt(discriminant)) / (2 * a));
  }

  auto intersection_points = std::vector<Point>();
  for (const auto &t : solutions) {
    const auto candidate_point = point_on_axis + normalized_axis * t;

    const auto vector_to_point = candidate_point - center_;
    const auto candidate_angle_global =
        std::atan2(vector_to_point.GetY(), vector_to_point.GetX());
    const auto start_angle =
        std::atan2(radius_vector_.GetY(), radius_vector_.GetX());

    auto candidate_angle = candidate_angle_global - start_angle;
    if (candidate_angle < start_angle) {
      candidate_angle += 2 * M_PI;
    }

    if (candidate_angle >= start_angle &&
        candidate_angle <= start_angle + angle_) {
      intersection_points.push_back(candidate_point);
    }
  }

  return intersection_points;
}

double Arc::DistanceToPoint(const Point &point) const {
  return 0; // TODO
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

} // namespace diamond_fem::geometry
