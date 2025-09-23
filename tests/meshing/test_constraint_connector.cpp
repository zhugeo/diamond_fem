#include <algorithm>
#include <gtest/gtest.h>
#include <ranges>

#include <geometry/arc.hpp>
#include <geometry/line.hpp>
#include <geometry/point.hpp>
#include <meshing/constraint_connector.hpp>
#include <meshing/mesh.hpp>
#include <vector>

namespace diamond_fem::meshing {

namespace {

PointWithBorderInfo
MakePointWithBorderInfo(double x, double y,
                        std::optional<internal::BorderRef> border) {
  return PointWithBorderInfo{geometry::Point{x, y}, border};
}

std::vector<Constraint> SortConstraints(std::vector<Constraint> constraints) {
  auto transformed_constraints =
      constraints | std::views::transform([](const auto &constraint) {
        return Constraint{
            .start_point_idx =
                std::max(constraint.start_point_idx, constraint.end_point_idx),
            .end_point_idx =
                std::min(constraint.start_point_idx, constraint.end_point_idx),
            .border_idx = constraint.border_idx,
        };
      }) |
      std::ranges::to<std::vector>();

  std::ranges::sort(
      transformed_constraints, [](const auto &lhs, const auto &rhs) {
        return std::make_tuple(lhs.start_point_idx, lhs.end_point_idx,
                               lhs.border_idx) <
               std::make_tuple(rhs.start_point_idx, rhs.end_point_idx,
                               rhs.border_idx);
      });
  return std::move(transformed_constraints);
}

TEST(TestConstraintConnector, ShouldConnectBorderPoints) {
  // given
  const auto borders = std::vector{
      std::make_shared<analysis_task::Border>(analysis_task::Border{
          .border_condition = nullptr,
          .curve = std::make_shared<geometry::Line>(geometry::Point{-1, 0},
                                                    geometry::Point{1, 0}),
      }),
      std::make_shared<analysis_task::Border>(analysis_task::Border{
          .border_condition = nullptr,
          .curve = std::make_shared<geometry::Arc>(geometry::Point{0, 0},
                                                   geometry::Vec{1, 0}, M_PI),
      }),
  };
  const auto points = std::vector{
      MakePointWithBorderInfo(-0.5, 0, borders[0]),
      MakePointWithBorderInfo(0.99, 0, borders[0]),
      MakePointWithBorderInfo(std::cos(M_PI / 4), std::sin(M_PI / 4),
                              borders[1]),
      MakePointWithBorderInfo(-std::cos(0.1), std::sin(0.1), borders[1]),
      MakePointWithBorderInfo(0, 0.5, std::nullopt),
  };
  const auto expected_constraints = SortConstraints(std::vector<Constraint>{
      {.start_point_idx = 1, .end_point_idx = 2, .border_idx = 0},
      {.start_point_idx = 0, .end_point_idx = 3, .border_idx = 1},
      {.start_point_idx = 0, .end_point_idx = 1, .border_idx = 0},
      {.start_point_idx = 2, .end_point_idx = 3, .border_idx = 1},
  });
  auto connector = ConstraintConnector(points, borders);

  // when
  const auto constraints = SortConstraints(connector.Connect());

  // then
  EXPECT_EQ(constraints.size(), expected_constraints.size());
  for (int i = 0; i < std::min(constraints.size(), expected_constraints.size());
       i++) {
    EXPECT_EQ(constraints[i].border_idx, expected_constraints[i].border_idx);
    EXPECT_EQ(constraints[i].start_point_idx,
              expected_constraints[i].start_point_idx);
    EXPECT_EQ(constraints[i].end_point_idx,
              expected_constraints[i].end_point_idx);
  }
}

} // namespace

} // namespace diamond_fem::meshing
