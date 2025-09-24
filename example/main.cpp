#include <memory>
#include <vector>

#include <CGAL/draw_triangulation_2.h>

#include <analysis_task/border.hpp>
#include <analysis_task/border_condition.hpp>
#include <geometry/arc.hpp>
#include <geometry/line.hpp>
#include <meshing/point_sparsener.hpp>
#include <meshing/point_stepper.hpp>
#include <meshing/triangulator.hpp>

namespace df = diamond_fem;
using namespace df::geometry;

int main() {
  const auto borders = std::vector{
      std::make_shared<const df::analysis_task::Border>(
          df::analysis_task::Border{
              .border_condition =
                  {.type =
                       df::analysis_task::BorderConditionType::ConstTemperature,
                   .value = 0},
              .curve = std::make_shared<Line>(Point(-5, 0), Point(0, 0)),
          }),
      std::make_shared<const df::analysis_task::Border>(
          df::analysis_task::Border{
              .border_condition =
                  {.type =
                       df::analysis_task::BorderConditionType::ConstTemperature,
                   .value = 0},
              .curve = std::make_shared<Line>(Point(0, -5), Point(0, 0)),
          }),
      std::make_shared<const df::analysis_task::Border>(
          df::analysis_task::Border{
              .border_condition =
                  {.type =
                       df::analysis_task::BorderConditionType::ConstTemperature,
                   .value = 0},
              .curve = std::make_shared<Line>(Point(-5, 0), Point(0, 5)),
          }),
      std::make_shared<const df::analysis_task::Border>(
          df::analysis_task::Border{
              .border_condition =
                  {.type =
                       df::analysis_task::BorderConditionType::ConstTemperature,
                   .value = 0},
              .curve = std::make_shared<Line>(Point(0, -5), Point(5, 0)),
          }),
      std::make_shared<const df::analysis_task::Border>(
          df::analysis_task::Border{
              .border_condition =
                  {.type =
                       df::analysis_task::BorderConditionType::ConstTemperature,
                   .value = 0},
              .curve = std::make_shared<Arc>(Point(0, 0), Vec(5, 0), M_PI / 2),
          }),
  };

  const auto sparse_steps = std::vector{
      df::meshing::SparsingPassParameters{
          .min_distance_to_border = 1.3,
          .max_distance_to_neighbor_point = 0.55,
          .num_neighbors = 3,
      },
  };

  constexpr auto TSL = 0.55;

  auto point_stepper = df::meshing::PointStepper(borders, TSL);
  const auto dense_points = point_stepper.Step();

  auto point_sparsener =
      df::meshing::PointSparsener(sparse_steps, borders, dense_points);
  const auto sparse_points = point_sparsener.Sparse();

  auto constraint_connector =
      df::meshing::ConstraintConnector(sparse_points, borders);
  const auto constraints = constraint_connector.Connect();

  auto triangulator =
      df::meshing::Triangulator(sparse_points, borders, constraints);
  triangulator.BuildTriangulation();
  const auto triangulation = triangulator.GetTriangulation();

  // Draw the triangulation
  CGAL::draw(triangulation);

  return 0;
}
