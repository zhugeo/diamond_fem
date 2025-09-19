#ifndef _DIAMOND_FEM_MESHING_MESH_HPP
#define _DIAMOND_FEM_MESHING_MESH_HPP

#include <vector>

namespace diamond_fem::meshing {

struct PointWithBorderInfo {
  geometry::Point point;
  std::optional<internal::BorderRef> border;
};

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_MESH_HPP
