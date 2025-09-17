#include <cmath>
#include <fmt/format.h>
#include <geometry/core.hpp>
#include <geometry/line.hpp>

namespace diamond_fem::geometry {

Line::Line(const Point &p1, const Point &p2) : p1_(p1), p2_(p2) {}

double Line::Length() const {
  return std::sqrt(std::pow(p1_.GetX() - p2_.GetX(), 2) +
                   std::pow(p1_.GetY() - p2_.GetY(), 2));
}

Point Line::GetParametricPoint(const double &t) const {
  const auto direction = p2_ - p1_;
  const auto direction_ort = direction.Normalized();
  return p1_ + t * direction_ort;
}

std::vector<Point>
Line::GetIntersectionsWithAxis(const Point &point_on_axis,
                               const Vec &axis_direction) const {
  const auto direction = (p2_ - p1_).Normalized();

  // Уравнение прямой: P = p1 + t * direction
  // Уравнение оси: Q = point_on_axis + s * axis_direction

  const auto A = direction.GetX();
  const auto B = -axis_direction.GetX();
  const auto C = direction.GetY();
  const auto D = -axis_direction.GetY();

  const auto E = point_on_axis.GetX() - p1_.GetX();
  const auto F = point_on_axis.GetY() - p1_.GetY();

  const auto det = A * D - B * C;

  if (std::abs(det) < EPSILON) { // Прямые параллельны
    return {};
  }

  const auto t = (D * E - B * F) / det;
  const auto s = (A * F - C * E) / det;

  Point intersection = p1_ + t * direction;
  if (IsInRange(intersection.GetX(), p1_.GetX(), p2_.GetX()) &&
      IsInRange(intersection.GetY(), p1_.GetY(), p2_.GetY()))
    return {intersection};

  return {};
}

double Line::DistanceToPoint(const Point &point) const {
  const auto direction = (p2_ - p1_).Normalized();
  const auto am = (point - p1_);
  const auto projection_of_am_to_line = am.DotProduct(direction) * direction;
  const auto distance_vector = am - projection_of_am_to_line;
  const auto nearest_point = point - distance_vector;

  if (IsInRange(nearest_point.GetX(), p1_.GetX(), p2_.GetX()) &&
      IsInRange(nearest_point.GetY(), p1_.GetY(), p2_.GetY())) {
    return distance_vector.Length();
  }
  return std::min(am.Length(), (p2_ - point).Length());
}

Vec Line::NormalAtPoint(const Point &point) const {
  const auto direction = p2_ - p1_;
  return Vec(direction.GetY(), -direction.GetX()).Normalized();
}

std::string Line::Description() const {
  return fmt::format("Line(Point({}, {}), Point({}, {}))", p1_.GetX(),
                     p1_.GetY(), p2_.GetX(), p2_.GetY());
}

} // namespace diamond_fem::geometry
