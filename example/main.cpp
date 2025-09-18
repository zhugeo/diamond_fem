#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include <analysis_task/border.hpp>
#include <analysis_task/border_condition.hpp>
#include <geometry/arc.hpp>
#include <geometry/line.hpp>
#include <meshing/point_stepper.hpp>

namespace df = diamond_fem;

int main() {
  auto borders = std::vector<std::shared_ptr<df::analysis_task::Border>>();

  borders.push_back(std::make_shared<df::analysis_task::Border>(
      std::make_shared<df::analysis_task::BorderConditionConstant>(0.0),
      std::make_shared<df::geometry::Line>(df::geometry::Point{0.0, 0.0},
                                           df::geometry::Point{1.0, 1.0})));

  auto stepper = df::meshing::PointStepper(std::move(borders));

  const auto points = stepper.Step();

  auto out = std::ofstream("out.txt");
  for (const auto &point : points) {
    out << point.point.GetX() << " " << point.point.GetY() << std::endl;
  }
  out.close();
}
