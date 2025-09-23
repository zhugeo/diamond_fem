#ifndef _DIAMOND_FEM_MESHING_CONSTRAINT_CONNECTOR_HPP
#define _DIAMOND_FEM_MESHING_CONSTRAINT_CONNECTOR_HPP

#include <unordered_map>
#include <vector>

#include <analysis_task/border.hpp>
#include <meshing/internal_boost_rtree.hpp>
#include <meshing/mesh.hpp>

namespace diamond_fem::meshing {

struct Constraint {
  int start_point_idx;
  int end_point_idx;
  int border_idx;
};

class ConstraintConnector {
public:
  ConstraintConnector(const std::vector<PointWithBorderInfo> &points,
                      const std::vector<internal::BorderRef> &borders);
  std::vector<Constraint> Connect();

private:
  void GroupPointsOnBorders_();
  void ConnectContiguousPoints_();
  void ConnectPointsOnOneBorder_(int border_idx);
  void IndexateBorderStartPoints_();
  void ConnectAdjacentBorders_();

  std::vector<PointWithBorderInfo> points_;
  std::vector<internal::BorderRef> borders_;

  std::unordered_map<internal::BorderRef, int> border_to_border_idx_;
  std::vector<std::vector<int>>
      points_on_border_; // points_on_border_[i][j] is an index j-th point
                         // (ordered ascending by parameter) on borders[i]

  std::vector<geometry::Point> border_start_points_;
  internal::RTreeWithBorderIndex rtree_;

  std::vector<Constraint> constraints_;
};

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_CONSTRAINT_CONNECTOR_HPP
