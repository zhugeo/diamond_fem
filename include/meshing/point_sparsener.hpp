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
 * - distance to neighbor point is more than min_distance_to_neighbor_point.
 */

#include <vector>

#include <geometry/geometry_fwd.hpp>

namespace diamond_fem::meshing {

struct SparsingPass {
  double min_distance_to_border;
  double min_distance_to_neighbor_point;
};

struct SparsingParameters {
  std::vector<SparsingPass> sparsing_passes;
};

class PointSparsener {
public:
  PointSparsener(double sparcing_radius);

  std::vector<Point> Sparce(const std::vector<Point> &points) const;

private:
  double sparcing_radius_;
};

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_POINT_SPARSENER_HPP
