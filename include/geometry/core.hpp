#ifndef _DIAMOND_FEM_GEOMETRY_CORE_HPP
#define _DIAMOND_FEM_GEOMETRY_CORE_HPP

#include <boost/geometry.hpp>

namespace diamond_fem::geometry {

constexpr double EPSILON = 1e-5;

bool IsNear(double a, double b);

bool IsInRange(double x, double a, double b);

using

} // namespace diamond_fem::geometry

#endif // _DIAMOND_FEM_GEOMETRY_CORE_HPP
