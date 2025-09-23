#include <gtest/gtest.h>

#include <algorithm>
#include <tuple>

#include <geometry/arc.hpp>
#include <geometry/point.hpp>

namespace diamond_fem::geometry {

namespace {

std::vector<Point> SortedPoints(std::vector<Point> points) {
  std::sort(points.begin(), points.end(), [](const Point &p1, const Point &p2) {
    return std::make_tuple(p1.GetX(), p1.GetY()) <
           std::make_tuple(p2.GetX(), p2.GetY());
  });

  return std::move(points);
}

TEST(TestArc, TestLength_ShouldReturnCorrectLength_ForQuarterCircle) {
  // given/when
  const auto center = Point(0, 0);
  const auto radius_vector = Vec(1, 0);
  const auto angle = M_PI / 2;
  const auto arc = Arc(center, radius_vector, angle);

  // then
  EXPECT_DOUBLE_EQ(arc.Length(), M_PI * 1.0 / 2.0);
}

TEST(TestArc, TestGetParametricPoint_ShouldReturnStartPoint) {
  // given
  const auto center = Point(0, 0);
  const auto radius_vector = Vec(1, 0);
  const auto angle = M_PI / 2;
  const auto arc = Arc(center, radius_vector, angle);

  // when
  const auto result = arc.GetParametricPoint(0);

  // then
  EXPECT_DOUBLE_EQ(result.GetX(), 1);
  EXPECT_DOUBLE_EQ(result.GetY(), 0);
}

TEST(TestArc, TestGetParametricPoint_ShouldReturnEndPoint) {
  // given
  const auto center = Point(0, 0);
  const auto radius_vector = Vec(1, 0);
  const auto angle = M_PI / 2;
  const auto arc = Arc(center, radius_vector, angle);

  // when
  const auto result = arc.GetParametricPoint(0);

  // then
  EXPECT_DOUBLE_EQ(result.GetX(), 1);
  EXPECT_DOUBLE_EQ(result.GetY(), 0);
}

TEST(TestArc,
     TestGetIntersectionsWithAxis_ShouldReturnEmpty_WhenLineOutsideArc) {
  // given
  const auto arc = Arc(Point(0, 0), Vec(5, 0), 2 * M_PI);

  const auto point_on_axis = Point(10, 0);
  const auto axis_direction = Vec(-1, 1);

  // when
  const auto result =
      arc.GetIntersectionsWithAxis(point_on_axis, axis_direction);

  // then
  EXPECT_EQ(result, std::vector<Point>{});
}

TEST(
    TestArc,
    TestGetIntersectionsWithAxis_ShouldReturnSinglePoint_WhenTangentInsideArcBounds) {
  // given
  const auto arc = Arc(Point(0, 0), Vec(5, 0), M_PI);

  const auto point_on_axis = Point(5, 0);
  const auto axis_direction = Vec(0, 1);

  auto expected = std::vector{Point(5, 0)};

  // when
  auto result = arc.GetIntersectionsWithAxis(point_on_axis, axis_direction);

  // then
  EXPECT_EQ(SortedPoints(std::move(result)), SortedPoints(std::move(expected)));
}

TEST(
    TestArc,
    TestGetIntersectionsWithAxis_ShouldReturnEmpty_WhenTangentOutsideArcBounds) {
  // given
  const auto arc = Arc(Point(0, 0), Vec(0, 5), M_PI);

  const auto point_on_axis = Point(5, 0);
  const auto axis_direction = Vec(0, 1);

  // when
  const auto result =
      arc.GetIntersectionsWithAxis(point_on_axis, axis_direction);

  // then
  EXPECT_EQ(result, std::vector<Point>{});
}

TEST(TestArc,
     TestGetIntersectionsWithAxis_ShouldReturnTwoPoints_WhenFullIntersection) {
  // given
  const auto arc = Arc(Point(0, 0), Vec(5, 0), 2 * M_PI);

  const auto point_on_axis = Point(0, 0);
  const auto axis_direction = Vec(1, 0);

  auto expected = std::vector{Point(-5, 0), Point(5, 0)};

  // when
  auto result = arc.GetIntersectionsWithAxis(point_on_axis, axis_direction);

  // then
  EXPECT_EQ(SortedPoints(std::move(result)), SortedPoints(std::move(expected)));
}

TEST(
    TestArc,
    TestGetIntersectionsWithAxis_ShouldReturnOnePoint_WhenPartialIntersection) {
  // given
  const auto arc = Arc(Point(0, 0), Vec(5, 0), M_PI);

  const auto point_on_axis = Point(0, 0);
  const auto axis_direction = Vec(4, 3);

  auto expected = std::vector{Point(4, 3)};

  // when
  auto result = arc.GetIntersectionsWithAxis(point_on_axis, axis_direction);

  // then
  EXPECT_EQ(SortedPoints(std::move(result)), SortedPoints(std::move(expected)));
}

TEST(
    TestArc,
    TestGetIntersectionsWithAxis_ShouldReturnEmpty_WhenBothPointsOutsideBounds) {
  // given
  const auto arc = Arc(Point(0, 0), Vec(5, 0), M_PI / 18);

  const auto point_on_axis = Point(0, 0);
  const auto axis_direction = Vec(4, 3);

  // when
  const auto result =
      arc.GetIntersectionsWithAxis(point_on_axis, axis_direction);

  // then
  EXPECT_EQ(result, std::vector<Point>{});
}

TEST(
    TestArc,
    TestGetIntersectionsWithAxis_ShouldReturnBoundaryPoints_WhenIntersectingAtEdges) {
  // given
  const auto arc = Arc(Point(0, 0), Vec(0, 5), M_PI);

  const auto point_on_axis = Point(0, 0);
  const auto axis_direction = Vec(0, 1);

  auto expected = std::vector{Point(0, 5), Point(0, -5)};

  // when
  auto result = arc.GetIntersectionsWithAxis(point_on_axis, axis_direction);

  // then
  EXPECT_EQ(SortedPoints(std::move(result)), SortedPoints(std::move(expected)));
}

TEST(TestArc, TestDistanceToPoint_ShouldReturnZero_ForPointOnArc) {
  // given/when
  const auto center = Point(0, 0);
  const auto radius_vector = Vec(1, 0);
  const auto angle = M_PI / 2;
  const auto arc = Arc(center, radius_vector, angle);
  const auto test_point = Point(0, 1);

  // when
  const auto distance = arc.DistanceToPoint(test_point);

  // then
  EXPECT_DOUBLE_EQ(distance, 0.0);
}

TEST(TestArc, TestDistanceToPoint_ShouldReturnRadius_ForPointOnCenter) {
  // given/when
  const auto center = Point(0, 0);
  const auto radius_vector = Vec(1, 0);
  const auto angle = M_PI / 2;
  const auto arc = Arc(center, radius_vector, angle);
  const auto test_point = Point(0, 0);

  // when
  const auto distance = arc.DistanceToPoint(test_point);

  // then
  EXPECT_DOUBLE_EQ(distance, 1.0);
}

TEST(TestArc, TestNormalAtPoint) {
  // given
  const auto center = Point(0, 0);
  const auto radius_vector = Vec(1, 0);
  const auto angle = M_PI / 2;
  const auto arc = Arc(center, radius_vector, angle);
  const auto test_point = Point(1, 0);

  // when
  const auto normal = arc.NormalAtPoint(test_point);

  // then
  EXPECT_DOUBLE_EQ(normal.GetX(), 1.0);
  EXPECT_DOUBLE_EQ(normal.GetY(), 0.0);
}

TEST(TestArc, TestDescription) {
  // given
  const auto center = Point(0, 0);
  const auto radius_vector = Vec(1, 0);
  const auto arc = Arc(center, radius_vector, M_PI / 2);

  // when
  const auto description = arc.Description();

  // then
  EXPECT_EQ(description, "Arc(Point(0, 0), Vec(1, 0), 1.5707963267948966)");
}

TEST(TestArc, TestGetPointParameter) {
  // given/when
  const auto center = Point(0, 0);
  const auto radius_vector = Vec(1, 0);
  const auto arc = Arc(center, radius_vector, M_PI);

  // then
  EXPECT_DOUBLE_EQ(arc.GetPointParameter(Point(1, 0)), 0.0);
  EXPECT_DOUBLE_EQ(arc.GetPointParameter(Point(0, 1)), M_PI / 2);
  EXPECT_DOUBLE_EQ(arc.GetPointParameter(Point(-1, 0)), M_PI);
}

} // namespace

} // namespace diamond_fem::geometry
