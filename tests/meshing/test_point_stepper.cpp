#include <algorithm>
#include <ranges>
#include <vector>

#include <gtest/gtest.h>

#include <geometry/geometry_fwd.hpp>

#include <analysis_task/border.hpp>
#include <geometry/core.hpp>
#include <geometry/line.hpp>
#include <geometry/point.hpp>
#include <meshing/point_stepper.hpp>

namespace diamond_fem::meshing {

namespace {

std::vector<geometry::Point> SortedResult(std::vector<geometry::Point> points) {
  std::ranges::sort(points, [](const auto &lhs, const auto &rhs) {
    return std::make_tuple(lhs.GetX(), lhs.GetY()) <
           std::make_tuple(rhs.GetX(), rhs.GetY());
  });

  return points;
}

const auto AssertResultIsExpected(std::vector<geometry::Point> result,
                                  std::vector<geometry::Point> expected) {
  ASSERT_EQ(result.size(), expected.size()) << "Result size is not as expected";

  const auto sorted_result = SortedResult(std::move(result));
  const auto sorted_expected = SortedResult(std::move(expected));
  for (int i = 0; i < sorted_result.size(); i++) {
    if (!geometry::PointsNear(sorted_expected[i], sorted_result[i])) {
      ASSERT_EQ(sorted_expected[i], sorted_result[i]);
    }
  }
}

TEST(TestPointStepper, TestPointStepper_ShouldStepRectangle) {
  // given
  using Border = const analysis_task::Border;
  const auto side_1 = std::make_shared<geometry::Line>(geometry::Point(1, 1),
                                                       geometry::Point(1, 4));
  const auto side_2 = std::make_shared<geometry::Line>(geometry::Point(1, 4),
                                                       geometry::Point(5, 4));
  const auto side_3 = std::make_shared<geometry::Line>(geometry::Point(5, 4),
                                                       geometry::Point(5, 1));
  const auto side_4 = std::make_shared<geometry::Line>(geometry::Point(5, 1),
                                                       geometry::Point(1, 1));

  auto borders = std::vector{std::make_shared<Border>(Border{.curve = side_1}),
                             std::make_shared<Border>(Border{.curve = side_2}),
                             std::make_shared<Border>(Border{.curve = side_3}),
                             std::make_shared<Border>(Border{.curve = side_4})};

  auto stepper = PointStepper(std::move(borders), 2.5);

  const auto expected_points =
      std::vector{geometry::Point(3.5, 1),
                  geometry::Point(1, 1),
                  geometry::Point(5, 1),
                  geometry::Point(3.5, 3.5),
                  geometry::Point(1, 3.5),
                  geometry::Point(5, 3.5),
                  geometry::Point(2.158949439651983, 4),
                  geometry::Point(4.608439182435161, 1),
                  geometry::Point(1.391560817564839, 1),
                  geometry::Point(3.8410505603480174, 4)};

  // when
  const auto points = stepper.Step();

  // then
  const auto pure_points =
      points |
      std::views::transform([](const auto &point) { return point.point; }) |
      std::ranges::to<std::vector>();

  for (const auto &point : pure_points) {
    EXPECT_TRUE(point.GetX() >= 1) << point;
    EXPECT_TRUE(point.GetX() <= 5) << point;
    EXPECT_TRUE(point.GetY() >= 1) << point;
    EXPECT_TRUE(point.GetY() <= 4) << point;
  }

  AssertResultIsExpected(pure_points, expected_points);
}

} // namespace

} // namespace diamond_fem::meshing
