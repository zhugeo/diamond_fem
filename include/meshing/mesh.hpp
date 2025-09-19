#ifndef _DIAMOND_FEM_MESHING_MESH_HPP
#define _DIAMOND_FEM_MESHING_MESH_HPP

#include <optional>

#include <geometry/geometry_fwd.hpp>

#include <analysis_task/border.hpp>
#include <geometry/point.hpp>

namespace diamond_fem::meshing {

namespace internal {

using BorderRef = std::shared_ptr<analysis_task::Border>;

} // namespace internal

struct PointWithBorderInfo {
  geometry::Point point;
  std::optional<internal::BorderRef> border;
};

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_MESH_HPP
