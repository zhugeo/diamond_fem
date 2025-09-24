#ifndef _DIAMOND_FEM_MESHING_POINT_SPARSENER_HPP
#define _DIAMOND_FEM_MESHING_POINT_SPARSENER_HPP

/**
 * After stepping, there are many points in mesh. But gradients of temperature
 * are not changed evenly on plate: areas close to border are more "dynamic"
 * than areas in the middle of the plate. To dramatically reduce elements count
 * with minor solution quality losses, there is sparsening step in mesh
 * generation.

 * Sparsening is done in several passes. Each pass removes points that meet two
 * following conditions:
 * - distance to border is more than min_distance_to_border;
 * - distance to `num_neighbors` neighbor points is less than
 max_distance_to_neighbor_point.
 */

#include <vector>

#include <meshing/internal_boost_rtree.hpp>
#include <meshing/mesh.hpp>

namespace diamond_fem::meshing {

namespace internal {

bool BoostPointsNear(const BoostPoint &p1, const BoostPoint &p2);

std::vector<PointWithBorderInfo>
SortPoints(std::vector<PointWithBorderInfo> points_to_sort);

struct PointWrapper {
  PointWithBorderInfo point_with_border_info;
  BoostPoint boost_point;
  double distance_to_border;
};

} // namespace internal

struct SparsingPassParameters {
  double min_distance_to_border;
  double max_distance_to_neighbor_point;
  int num_neighbors;
};

class PointSparsener {
public:
  PointSparsener(
      const std::vector<SparsingPassParameters> &sparsing_passes_parameters,
      const std::vector<internal::BorderRef> &borders,
      std::vector<PointWithBorderInfo> points);

  std::vector<PointWithBorderInfo> Sparse();

private:
  void RebuildRTree_();
  void DoSparsingPass_(const SparsingPassParameters &pass_parameters);
  bool ShouldRemovePoint_(const internal::PointWrapper &point,
                          const SparsingPassParameters &pass_parameters);

  std::vector<SparsingPassParameters> parameters_;
  std::vector<internal::PointWrapper> points_;
  internal::RTree rtree_;
};

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_POINT_SPARSENER_HPP
