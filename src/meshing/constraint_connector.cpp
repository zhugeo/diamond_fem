#include <meshing/constraint_connector.hpp>

namespace diamond_fem::meshing {

ConstraintConnector::ConstraintConnector(
    const std::vector<PointWithBorderInfo> &points,
    const std::vector<internal::BorderRef> &borders) {
  points_ = points;
  borders_ = borders;
}

} // namespace diamond_fem::meshing
