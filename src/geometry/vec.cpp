#include <geometry/vec.hpp>

#include <cmath>
#include <fmt/format.h>

#include <geometry/point.hpp>

namespace diamond_fem::geometry {

Vec::Vec(double x, double y) : x_(x), y_(y) {}

double Vec::GetX() const { return x_; }

double Vec::GetY() const { return y_; }

double Vec::Length() const { return std::sqrt(x_ * x_ + y_ * y_); }

Vec Vec::Normalized() const { return *this * (1 / Length()); }

double Vec::DotProduct(const Vec &other) const {
  return x_ * other.x_ + y_ * other.y_;
}

bool Vec::operator==(const Vec &other) const {
  return x_ == other.x_ && y_ == other.y_;
}

bool Vec::operator!=(const Vec &other) const { return !(*this == other); }

Vec Vec::operator+(const Vec &other) const {
  return Vec(x_ + other.x_, y_ + other.y_);
}

Vec Vec::operator-(const Vec &other) const {
  return Vec(x_ - other.x_, y_ - other.y_);
}

Vec Vec::operator*(double scalar) const {
  return Vec(x_ * scalar, y_ * scalar);
}

Vec operator*(double scalar, const Vec &vec) { return vec * scalar; }

std::ostream &operator<<(std::ostream &os, const Vec &vec) {
  os << fmt::format("Vec({}, {})", vec.x_, vec.y_);
  return os;
}

} // namespace diamond_fem::geometry
