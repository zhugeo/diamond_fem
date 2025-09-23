#ifndef _DIAMOND_FEM_MESHING_CONSTRAINT_CONNECTOR_HPP
#define _DIAMOND_FEM_MESHING_CONSTRAINT_CONNECTOR_HPP

#include <vector>

#include <analysis_task/border.hpp>
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

private:
  std::vector<PointWithBorderInfo> points_;
  std::vector<internal::BorderRef> borders_;
};

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_CONSTRAINT_CONNECTOR_HPP
