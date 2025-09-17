#ifndef _DIAMOND_FEM_MESHING_POINT_STEPPER_HPP
#define _DIAMOND_FEM_MESHING_POINT_STEPPER_HPP

#include <memory>

#include <geometry/geometry_fwd.hpp>

#include <analysis_task/border.hpp>
#include <geometry/point.hpp>

namespace diamond_fem::meshing {

struct PointWithBorderInfo {
  geometry::Point point;
  std::shared_ptr<analysis_task::Border> borders;
};

class PointStepper {
public:
  PointStepper(std::vector<std::shared_ptr<analysis_task::Border>> borders);
  std::vector<PointWithBorderInfo> Step();

private:
  std::vector<analysis_task::Border> borders_;
};

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_POINT_STEPPER_HPP
