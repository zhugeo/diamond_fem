#ifndef _DIAMOND_FEM_MESHING_TRIANGULATOR_HPP
#define _DIAMOND_FEM_MESHING_TRIANGULATOR_HPP

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/draw_triangulation_2.h>

#include <geometry/spatial_index.hpp>
#include <meshing/constraint_connector.hpp>
#include <meshing/mesh.hpp>

namespace diamond_fem::meshing {

namespace internal {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Triangulation = CGAL::Constrained_Delaunay_triangulation_2<K>;
using CGALPoint = Triangulation::Point;

using Unconstrained_Triangulation = CGAL::Delaunay_triangulation_2<K>;

enum class RawTriangleState { kInner, kOuter, kUnknown };

struct RawTriangle {
  int p_1_idx;
  int p_2_idx;
  int p_3_idx;
  RawTriangleState state = RawTriangleState::kUnknown;
};

} // namespace internal

class Triangulator {
public:
  Triangulator(std::vector<PointWithBorderInfo> points,
               std::vector<internal::BorderRef> borders,
               std::vector<Constraint> constraints);

  void BuildTriangulation();
  internal::Triangulation GetTriangulation() const;
  std::vector<MeshTriangle> ExtractTriangles();

private:
  void LoadConstraints_();
  void LoadPoints_();
  void ExtractTrianglesFromTriangulation_();
  void ApplyConstraintsToTriangles_();
  void FillRTreeIndex_();
  int GetPointIndex_(const geometry::Point &point) const;
  void IndexTriangles_();

  std::vector<PointWithBorderInfo> points_;
  std::vector<internal::BorderRef> borders_;
  std::vector<Constraint> constraints_;
  std::vector<MeshTriangle> triangles_;
  internal::Triangulation triangulation_;
  std::vector<std::vector<int>> triangles_adjacent_to_point_;
  geometry::SpatialIndex<int> rtree_;
};

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_TRIANGULATOR_HPP
