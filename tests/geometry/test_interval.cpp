#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include <geometry/geometry_fwd.hpp>

#include <geometry/core.hpp>
#include <geometry/interval.hpp>
#include <geometry/point.hpp>
#include <geometry/vec.hpp>

namespace diamond_fem::geometry {

namespace {

std::vector<Point> SortedResult(std::vector<Point> points) {
  std::ranges::sort(points, [](const auto &lhs, const auto &rhs) {
    return std::make_tuple(lhs.GetX(), lhs.GetY()) <
           std::make_tuple(rhs.GetX(), rhs.GetY());
  });

  return points;
}

const auto AssertResultIsExpected(std::vector<Point> result,
                                  std::vector<Point> expected) {
  ASSERT_EQ(result.size(), expected.size()) << "Result size is not as expected";

  const auto sorted_result = SortedResult(std::move(result));
  const auto sorted_expected = SortedResult(std::move(expected));
  for (int i = 0; i < sorted_result.size(); i++) {
    if (!PointsNear(sorted_expected[i], sorted_result[i])) {
      ASSERT_EQ(sorted_expected[i], sorted_result[i]);
    }
  }
}

TEST(TestInterval, HorizontalLineSegment) {
  // given
  const auto start_point = Point(0.6, 0);
  const auto end_point = Point(5.6, 0);
  const auto grid_node = Point(0, 0);
  const auto direction_step = Vec(1, 0);

  const auto interval =
      Interval(start_point, end_point, grid_node, direction_step);

  auto expected_points = std::vector{Point(1, 0), Point(2, 0), Point(3, 0),
                                     Point(4, 0), Point(5, 0)};

  // when
  auto result = interval.DoStepping();

  // then
  AssertResultIsExpected(std::move(result), std::move(expected_points));
}

TEST(TestInterval, VerticalLineSegment) {
  // given
  const auto start_point = Point(0, 0.3);
  const auto end_point = Point(0, 5.3);
  const auto grid_node = Point(0, 0);
  const auto direction_step = Vec(0, 1);

  const auto interval =
      Interval(start_point, end_point, grid_node, direction_step);

  auto expected_points = std::vector{Point(0, 1), Point(0, 2), Point(0, 3),
                                     Point(0, 4), Point(0, 5)};

  // when
  auto result = interval.DoStepping();

  // then
  AssertResultIsExpected(std::move(result), std::move(expected_points));
}

TEST(TestInterval, DiagonalLineSegment) {
  // given
  const auto start_point = Point(0.1, 0.1);
  const auto end_point = Point(4.8, 4.8);
  const auto grid_node = Point(0, 0);
  const auto direction_step = Vec(1, 1);

  const auto interval =
      Interval(start_point, end_point, grid_node, direction_step);
  auto expected_points =
      std::vector{Point(1, 1), Point(2, 2), Point(3, 3), Point(4, 4)};

  // when
  auto result = interval.DoStepping();

  // then
  AssertResultIsExpected(std::move(result), std::move(expected_points));
}

TEST(TestInterval, GridNodeOnSegment) {
  // given
  const auto start_point = Point(0, 0);
  const auto end_point = Point(4, 0);
  const auto grid_node = Point(2, 0); // On the segment
  const auto direction_step = Vec(1, 0);

  const auto interval =
      Interval(start_point, end_point, grid_node, direction_step);

  auto expected_points = std::vector{Point(0, 0), Point(1, 0), Point(2, 0),
                                     Point(3, 0), Point(4, 0)};

  // when
  auto result = interval.DoStepping();

  // then
  AssertResultIsExpected(std::move(result), std::move(expected_points));
}

TEST(TestInterval, ReverseDirectionStep) {
  // given
  const auto start_point = Point(0.2, -0.2);
  const auto end_point = Point(3.2, -0.2);
  const auto grid_node = Point(0, -0.2);
  const auto direction_step = Vec(-1, 0); // Negative direction

  const auto interval =
      Interval(start_point, end_point, grid_node, direction_step);

  auto expected_points =
      std::vector{Point(1, -0.2), Point(2, -0.2), Point(3, -0.2)};

  // when
  auto result = interval.DoStepping();

  // then
  AssertResultIsExpected(std::move(result), std::move(expected_points));
}

TEST(TestInterval, SinglePointSegment) {
  // given
  const auto start_point = Point(2, 3);
  const auto end_point = Point(2, 3);
  const auto grid_node = Point(0, 3);
  const auto direction_step = Vec(1, 0);

  const auto interval =
      Interval(start_point, end_point, grid_node, direction_step);

  auto expected_points = std::vector<Point>{Point(2, 3)};

  // when
  auto result = interval.DoStepping();

  // then
  AssertResultIsExpected(std::move(result), std::move(expected_points));
}

TEST(TestInterval, GridNodesNotFitToSegment) {
  // given
  const auto start_point = Point(1.1, 1.1);
  const auto end_point = Point(1.9, 1.9);
  const auto grid_node = Point(0, 0);
  const auto direction_step = Vec(1, 1);

  const auto interval =
      Interval(start_point, end_point, grid_node, direction_step);

  auto expected_points = std::vector<Point>{};

  // when
  auto result = interval.DoStepping();

  // then
  AssertResultIsExpected(std::move(result), std::move(expected_points));
}

} // namespace

} // namespace diamond_fem::geometry
