#include <iostream>
#include <memory>

#include <analysis_task/border.hpp>
#include <analysis_task/border_condition.hpp>
#include <geometry/arc.hpp>
#include <geometry/line.hpp>
#include <meshing/point_stepper.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/draw_polygon_2.h>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;

namespace df = diamond_fem;

int main() {
  auto borders = std::vector<std::shared_ptr<df::analysis_task::Border>>();

  borders.push_back(std::make_shared<df::analysis_task::Border>(
      std::make_shared<df::analysis_task::BorderConditionConstant>(0.0),
      std::make_shared<df::geometry::Line>(df::geometry::Point{0.0, 0.0},
                                           df::geometry::Point{1.0, 1.0})));

  auto stepper = df::meshing::PointStepper(std::move(borders));

  // const auto points = stepper.Step();
  Polygon_2 polygon;

  polygon.push_back(Point_2(0, 0));
  polygon.push_back(Point_2(1, 0));
  polygon.push_back(Point_2(1, 1));
  polygon.push_back(Point_2(0, 1));

  // Визуализация точек
  CGAL::draw(polygon);
}
