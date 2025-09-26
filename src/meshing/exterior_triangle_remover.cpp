#include <meshing/exterior_triangle_remover.hpp>

namespace diamond_fem::meshing {

ExteriorTriangleRemover::ExteriorTriangleRemover(
    std::vector<MeshTriangle> mesh_triangles) {
  // TODO
}

std::vector<MeshTriangle> ExteriorTriangleRemover::ExtractFilteredTriangles() {
  // TODO
  return {};
}

void ExteriorTriangleRemover::IndexTriangles_() {
  triangles_adjacent_to_point_.resize(points_.size());
  for (int triangle_idx = 0; triangle_idx < triangles_.size(); triangle_idx++) {
    const auto &triangle = triangles_[triangle_idx];
    const auto vertex_indices = std::vector{
        triangle.p_1_idx,
        triangle.p_2_idx,
        triangle.p_3_idx,
    };
    std::ranges::for_each(vertex_indices, [&](const auto &point_idx) {
      triangles_adjacent_to_point_[point_idx].push_back(triangle_idx);
    });
  }
}

} // namespace diamond_fem::meshing
