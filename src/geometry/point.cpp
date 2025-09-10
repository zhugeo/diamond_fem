#include <geometry/point.hpp>

#include <cmath>
#include <fmt/format.h>

#include <geometry/vec.hpp>

namespace diamond_fem::geometry {

Point::Point(double x, double y) : x_(x), y_(y) {}

double Point::GetX() const { return x_; }

double Point::GetY() const { return y_; }

bool Point::operator==(const Point &other) const {
  return (x_ == other.x_) && (y_ == other.y_);
}

bool Point::operator!=(const Point &other) const { return !(*this == other); }

Vec Point::operator-(const Point &other) const {
  return Vec(x_ - other.x_, y_ - other.y_);
}

Point Point::operator+(const Vec &vec) const {
  return Point(x_ + vec.GetX(), y_ + vec.GetY());
}
Point Point::operator-(const Vec &vec) const {
  return Point(x_ - vec.GetX(), y_ - vec.GetY());
}

double Point::DistanceTo(const Point &other) const {
  double dx = x_ - other.x_;
  double dy = y_ - other.y_;
  return std::sqrt(dx * dx + dy * dy);
}

std::ostream &operator<<(std::ostream &os, const Point &point) {
  os << fmt::format("Point({}, {})", point.x_, point.y_);
  return os;
}

} // namespace diamond_fem::geometry
