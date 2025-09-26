#include <ranges>
#include <vector>

#include <gtest/gtest.h>

#include <geometry/geometry_fwd.hpp>

#include <analysis_task/border.hpp>
#include <geometry/core.hpp>
#include <geometry/line.hpp>
#include <geometry/point.hpp>
#include <meshing/mesh.hpp>
#include <meshing/point_sparsener.hpp>
#include <meshing/point_stepper.hpp>

namespace diamond_fem::meshing {

namespace {

using geometry::Point;

std::vector<Point>
ExtractPoints(const std::vector<PointWithBorderInfo> &points_with_border_info) {
  return points_with_border_info |
         std::views::transform([](const auto &point_with_border_info) {
           return point_with_border_info.point;
         }) |
         std::ranges::to<std::vector>();
}

TEST(TestPointSparsener, ShouldSparsenPoints) {
  // given
  const auto points = std::vector<PointWithBorderInfo>{
      PointWithBorderInfo{Point{0, 1}, std::nullopt},
      PointWithBorderInfo{Point{1, 1}, std::nullopt},
      PointWithBorderInfo{Point{2, 1}, std::nullopt},
      PointWithBorderInfo{Point{3, 1}, std::nullopt},
      PointWithBorderInfo{Point{4, 1}, std::nullopt},
  };
  const auto pass_params = SparsingPassParameters{
      .min_distance_to_border = 0.1,
      .max_distance_to_neighbor_point = 1.0,
      .num_neighbors = 2,
  };
  auto point_sparsener = PointSparsener({pass_params}, {}, points);

  const auto expected_points = std::vector{points[0], points[2], points[4]};

  // when
  const auto sparsed_points = point_sparsener.Sparse();

  // then
  ASSERT_EQ(ExtractPoints(sparsed_points), ExtractPoints(expected_points));
}

TEST(TestPointSparsener, ShouldPreservePointsNearBorder) {
  // given
  const auto points = std::vector<PointWithBorderInfo>{
      PointWithBorderInfo{Point{0.5, 1}, std::nullopt},
      PointWithBorderInfo{Point{0.5, 2}, std::nullopt},
      PointWithBorderInfo{Point{0.5, 3}, std::nullopt},
      PointWithBorderInfo{Point{0.5, 4}, std::nullopt},
      PointWithBorderInfo{Point{1.1, 1.5}, std::nullopt},
  };
  const auto borders = std::vector{
      std::make_shared<const analysis_task::Border>(analysis_task::Border{
          .curve = std::make_shared<geometry::Line>(Point(0, 0), Point(0, 4))}),
  };
  const auto pass_params = SparsingPassParameters{
      .min_distance_to_border = 1,
      .max_distance_to_neighbor_point = 1.0,
      .num_neighbors = 2,
  };
  auto point_sparsener = PointSparsener({pass_params}, borders, points);

  const auto expected_points =
      std::vector{points[0], points[1], points[2], points[3]};

  // when
  const auto sparsed_points = point_sparsener.Sparse();

  // then
  ASSERT_EQ(ExtractPoints(sparsed_points), ExtractPoints(expected_points));
}

} // namespace

} // namespace diamond_fem::meshing
