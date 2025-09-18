#include <meshing/point_stepper.hpp>

namespace diamond_fem::meshing {

PointStepper::PointStepper(
    std::vector<std::shared_ptr<analysis_task::Border>> borders)
    : borders_(borders) {}

std::vector<PointWithBorderInfo> PointStepper::Step() {
  return {}; // TODO
}

} // namespace diamond_fem::meshing
