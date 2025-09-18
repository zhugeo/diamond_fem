#include <ranges>
#include <vector>

#include <gtest/gtest.h>

#include <geometry/geometry_fwd.hpp>

#include <analysis_task/border.hpp>
#include <geometry/line.hpp>
#include <geometry/point.hpp>
#include <meshing/point_stepper.hpp>

namespace diamond_fem::meshing {

TEST(TestPointStepper, TestPointStepper_ShouldStepRectangle) {
  // given
  using Border = analysis_task::Border;
  const auto side_1 = std::make_shared<geometry::Line>(geometry::Point(1, 1),
                                                       geometry::Point(1, 4));
  const auto side_2 = std::make_shared<geometry::Line>(geometry::Point(1, 4),
                                                       geometry::Point(4, 5));
  const auto side_3 = std::make_shared<geometry::Line>(geometry::Point(4, 5),
                                                       geometry::Point(5, 1));
  const auto side_4 = std::make_shared<geometry::Line>(geometry::Point(5, 1),
                                                       geometry::Point(1, 1));

  auto borders = std::vector{std::make_shared<Border>(Border{.curve = side_1}),
                             std::make_shared<Border>(Border{.curve = side_2}),
                             std::make_shared<Border>(Border{.curve = side_3}),
                             std::make_shared<Border>(Border{.curve = side_4})};

  auto stepper = PointStepper(std::move(borders), 1.0);

  const auto expected_points = std::vector{
      geometry::Point(1, 1), geometry::Point(2, 2), geometry::Point(3, 3),
      geometry::Point(4, 4), geometry::Point(5, 5)};

  // when
  const auto points = stepper.Step();

  // then
  const auto pure_points =
      points |
      std::views::transform([](const auto &point) { return point.point; }) |
      std::ranges::to<std::vector>();

  EXPECT_EQ(pure_points.size(), 16);
  EXPECT_EQ(pure_points, expected_points);
}

} // namespace diamond_fem::meshing
