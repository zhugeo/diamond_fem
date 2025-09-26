#ifndef _DIAMOND_FEM_EXTERIOR_TRIANGLE_REMOVER_HPP
#define _DIAMOND_FEM_EXTERIOR_TRIANGLE_REMOVER_HPP

#include <vector>

#include <geometry/point.hpp>
#include <meshing/mesh.hpp>

namespace diamond_fem::meshing {

namespace internal {

enum class TriangleState { kUnknown, kInternal, kExternal };

} // namespace internal

class ExteriorTriangleRemover {
public:
  ExteriorTriangleRemover(std::vector<MeshTriangle> mesh_triangles);
  std::vector<MeshTriangle> ExtractFilteredTriangles();

private:
  void IndexTriangles_();

  std::vector<MeshTriangle> triangles_;
  std::vector<geometry::Point> points_;
  std::vector<internal::TriangleState> triangle_states_;
};

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_EXTERIOR_TRIANGLE_REMOVER_HPP
