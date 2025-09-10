#include <geometry/constants.hpp>
#include <geometry/line.hpp>
#include <gtest/gtest.h>

namespace diamond_fem::geometry {

namespace {

bool IsNear(const double &a, const double &b) {
  return std::abs(a - b) < EPSILON;
}

TEST(TestLine, TestLength) {
  // given/when
  Point p1(0, 0);
  Point p2(3, 4);
  Line line(p1, p2);

  // then
  EXPECT_EQ(line.Length(), 5);
}

TEST(TestLine, TestGetParametricPoint) {
  // given/when
  const auto line = Line(Point(0, 0), Point(2, 2));

  // then
  EXPECT_TRUE(line.GetParametricPoint(0) == Point(0, 0));
  EXPECT_TRUE(line.GetParametricPoint(line.Length()) == Point(2, 2));
  EXPECT_TRUE(line.GetParametricPoint(line.Length() / 2) == Point(1, 1));
}

TEST(
    TestLine,
    TestGetIntersectionsWithAxis_ShouldReturnIntersectionPointWhenPointOnLineRange) {
  // given
  const auto line = Line(Point(0, 0), Point(2, 2));
  const auto point_on_axis = Point(2, 0);
  const auto axis_direction = Vec(1, -1);

  // when
  const auto intersection =
      line.GetIntersectionsWithAxis(point_on_axis, axis_direction);

  // then
  EXPECT_EQ(intersection, std::vector{Point(1, 1)});
}

TEST(
    TestLine,
    TestGetIntersectionsWithAxis_ShouldNotReturnIntersectionPoint_WhenPointNotOnLineRange) {
  // given
  const auto line = Line(Point(0, 0), Point(2, 2));
  const auto point_on_axis = Point(5, 0);
  const auto axis_direction = Vec(1, -1);

  // when
  const auto intersection =
      line.GetIntersectionsWithAxis(point_on_axis, axis_direction);

  // then
  EXPECT_TRUE(intersection.empty());
}

TEST(
    TestLine,
    TestGetIntersectionsWithAxis_ShouldNotReturnIntersectionPoint_WhenLineAndAxisCoincide) {
  // given
  const auto line = Line(Point(0, 0), Point(2, 2));
  const auto point_on_axis = Point(0, 0);
  const auto axis_direction_1 = Vec(1, 1);
  const auto axis_direction_2 = Vec(-1, -1);

  // when
  const auto intersection_1 =
      line.GetIntersectionsWithAxis(point_on_axis, axis_direction_1);
  const auto intersection_2 =
      line.GetIntersectionsWithAxis(point_on_axis, axis_direction_2);

  // then
  EXPECT_TRUE(intersection_1.empty());
  EXPECT_TRUE(intersection_2.empty());
}

TEST(
    TestLine,
    TestDistanceToPoint_ShouldReturnDistanceToLine_WhenProjectionPointIsOnLineRange) {
  // given
  const auto line = Line(Point(0, 0), Point(2, 2));
  const auto point = Point(2, 0);

  // when
  const auto distance = line.DistanceToPoint(point);

  // then
  EXPECT_TRUE(IsNear(distance, std::sqrt(2)));
}

TEST(
    TestLine,
    TestDistanceToPoint_ShouldReturnDistanceToNearestEndpoint_WhenProjectionPointIsNotOnLineRange) {
  // given
  const auto line = Line(Point(0, 0), Point(2, 2));
  const auto point = Point(-2, -2);

  // when
  const auto distance = line.DistanceToPoint(point);

  // then
  EXPECT_TRUE(IsNear(distance, std::sqrt(8)));
}

TEST(TestLine, TestNormalAtPoint) {
  // given
  const auto line = Line(Point(0, 0), Point(1, 1));

  // when
  const auto normal = line.NormalAtPoint(Point(0, 0));
  const auto diff = normal - Vec(1, -1).Normalized();

  // then
  EXPECT_TRUE(IsNear(diff.Length(), 0));
}

TEST(TestLine, TestDescription) {
  // given/when
  const auto line = Line(Point(0, 0), Point(1, 1));

  // then
  EXPECT_EQ(line.Description(), "Line(Point(0, 0), Point(1, 1))");
}

} // namespace

} // namespace diamond_fem::geometry
