#ifndef _DIAMOND_FEM_GEOMETRY_VEC_HPP
#define _DIAMOND_FEM_GEOMETRY_VEC_HPP

#include <geometry/geometry_fwd.hpp>
#include <ostream>

namespace diamond_fem::geometry {

class Vec {
public:
  explicit Vec(double x, double y);

  double GetX() const;
  double GetY() const;
  double Length() const;

  Vec Normalized() const;

  double DotProduct(const Vec &other) const;

  bool operator==(const Vec &other) const;
  bool operator!=(const Vec &other) const;

  Vec operator+(const Vec &other) const;
  Vec operator-(const Vec &other) const;
  Vec operator*(double scalar) const;

  friend Vec operator*(double scalar, const Vec &vec);
  friend std::ostream &operator<<(std::ostream &os, const Vec &vec);

private:
  double x_;
  double y_;
};

} // namespace diamond_fem::geometry

#endif // _DIAMOND_FEM_GEOMETRY_VEC_HPP
