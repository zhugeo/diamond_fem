#ifndef _DIAMOND_FEM_MESHING_MESH_HPP
#define _DIAMOND_FEM_MESHING_MESH_HPP

#include <optional>
#include <vector>

#include <geometry/geometry_fwd.hpp>

#include <analysis_task/border.hpp>
#include <analysis_task/border_condition.hpp>
#include <geometry/point.hpp>
#include <geometry/vec.hpp>

namespace diamond_fem::meshing {

namespace internal {

using BorderRef = std::shared_ptr<const analysis_task::Border>;

} // namespace internal

struct PointWithBorderInfo {
  geometry::Point point;
  std::optional<internal::BorderRef> border;
};

struct OuterEdgePayload {
  geometry::Vec normal_outside;
  analysis_task::BorderCondition border_condition;
};

struct MeshTriangle {
  int p_1_idx;
  int p_2_idx;
  int p_3_idx;
  std::optional<OuterEdgePayload> edge_p1_p2;
  std::optional<OuterEdgePayload> edge_p2_p3;
  std::optional<OuterEdgePayload> edge_p3_p1;
};

struct Mesh {
  std::vector<geometry::Point> points;
  std::vector<MeshTriangle> triangles;
};

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_MESH_HPP
