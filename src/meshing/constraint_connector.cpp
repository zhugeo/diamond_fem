#include <meshing/constraint_connector.hpp>

#include <ranges>
#include <stdexcept>

#include <fmt/format.h>

#include <geometry/core.hpp>
#include <geometry/curve.hpp>
#include <geometry/vec.hpp>
#include <meshing/internal_boost_rtree.hpp>

namespace diamond_fem::meshing {

ConstraintConnector::ConstraintConnector(
    const std::vector<PointWithBorderInfo> &points,
    const std::vector<internal::BorderRef> &borders) {
  points_ = points;
  borders_ = borders;
}

std::vector<Constraint> ConstraintConnector::Connect() {
  GroupPointsOnBorders_();
  ConnectContiguousPoints_();
  IndexateBorderStartAndEndPoints_();
  ConnectAdjacentBorders_();

  return std::move(constraints_);
}

void ConstraintConnector::GroupPointsOnBorders_() {
  for (auto border_idx = 0; border_idx < borders_.size(); border_idx++) {
    border_to_border_idx_[borders_[border_idx]] = border_idx;
  }

  points_on_border_.resize(borders_.size());

  for (auto point_idx = 0; point_idx < points_.size(); point_idx++) {
    if (points_[point_idx].border.has_value()) {
      const auto border_idx =
          border_to_border_idx_[points_[point_idx].border.value()];
      points_on_border_[border_idx].push_back(point_idx);
    }
  }

  for (auto border_idx = 0; border_idx < borders_.size(); border_idx++) {
    const auto &border_curve = *(borders_[border_idx]->curve);
    std::ranges::sort(
        points_on_border_[border_idx], [&](const auto &lhs, const auto &rhs) {
          return border_curve.GetPointParameter(points_[lhs].point) <
                 border_curve.GetPointParameter(points_[rhs].point);
        });
  }
}

void ConstraintConnector::ConnectContiguousPoints_() {
  for (auto border_idx = 0; border_idx < borders_.size(); border_idx++) {
    ConnectPointsOnOneBorder_(border_idx);
  }
}

void ConstraintConnector::ConnectPointsOnOneBorder_(int border_idx) {
  for (auto i = 0; i < points_on_border_[border_idx].size() - 1; i++) {
    constraints_.push_back(Constraint{
        .start_point_idx = points_on_border_[border_idx][i],
        .end_point_idx = points_on_border_[border_idx][i + 1],
        .border_idx = border_idx,
    });
  }
}

void ConstraintConnector::IndexateBorderStartAndEndPoints_() {
  for (auto border_idx = 0; border_idx < borders_.size(); border_idx++) {
    const auto &curve = *(borders_[border_idx]->curve);
    const auto start_point = curve.GetParametricPoint(0);
    const auto end_point = curve.GetParametricPoint(curve.Length());

    const auto boost_start_point =
        internal::BoostPoint{start_point.GetX(), start_point.GetY()};
    const auto boost_end_point =
        internal::BoostPoint{end_point.GetX(), end_point.GetY()};

    rtree_.insert(std::make_pair(
        boost_start_point,
        internal::BorderInfo{.border_idx = border_idx,
                             .type = internal::PointType::kStart}));
    rtree_.insert(std::make_pair(
        boost_end_point,
        internal::BorderInfo{.border_idx = border_idx,
                             .type = internal::PointType::kEnd}));
  }
}

void ConstraintConnector::ConnectAdjacentBorders_() {
  for (auto border_idx = 0; border_idx < borders_.size(); border_idx++) {
    const auto &curve = *(borders_[border_idx]->curve);

    const auto start_point = curve.GetParametricPoint(0);
    const auto end_point = curve.GetParametricPoint(curve.Length());

    ConnectAdjacentBordersToPoint_(start_point, border_idx,
                                   internal::PointType::kStart);
    ConnectAdjacentBordersToPoint_(end_point, border_idx,
                                   internal::PointType::kEnd);
  }
}

void ConstraintConnector::ConnectAdjacentBordersToPoint_(
    const geometry::Point &point, int border_idx,
    internal::PointType point_type) {
  const auto search_box = internal::bg::model::box<internal::BoostPoint>{
      internal::BoostPoint{point.GetX() - geometry::EPSILON,
                           point.GetY() - geometry::EPSILON},
      internal::BoostPoint{point.GetX() + geometry::EPSILON,
                           point.GetY() + geometry::EPSILON},
  };

  auto overlapping_points_with_template_point =
      std::vector<internal::BoostPointWithBorderInfo>{};
  rtree_.query(internal::bgi::intersects(search_box),
               std::back_inserter(overlapping_points_with_template_point));

  const auto overlapping_points =
      overlapping_points_with_template_point |
      std::views::filter([&](const auto &point) {
        return point.second.type != point_type ||
               point.second.border_idx != border_idx;
      }) |
      std::ranges::to<std::vector>();

  if (overlapping_points.size() != 1) {
    const auto &curve = *(borders_[border_idx]->curve);
    throw std::runtime_error(
        fmt::format("Found {} points overlapping the end point "
                    "for border {}",
                    overlapping_points.size(), curve.Description()));
  }
  const auto overlapped_point = overlapping_points[0];
  const auto overlapped_border_idx = overlapped_point.second.border_idx;

  // Prevent two same constraints
  if (overlapped_border_idx > border_idx) {
    return;
  }

  // Allow full circles to connect, but prevent incorrect constraints here
  if (overlapped_border_idx == border_idx) {
    if (overlapped_point.second.type == point_type) {
      return;
    }
    if (point_type == internal::PointType::kStart) {
      return;
    }
  }

  const auto nearest_border_1_point_idx =
      point_type == internal::PointType::kStart
          ? points_on_border_[border_idx].front()
          : points_on_border_[border_idx].back();
  const auto nearest_border_1_point = points_[nearest_border_1_point_idx].point;

  const auto nearest_border_2_point_idx =
      overlapped_point.second.type == internal::PointType::kStart
          ? points_on_border_[overlapped_border_idx].front()
          : points_on_border_[overlapped_border_idx].back();
  const auto nearest_border_2_point = points_[nearest_border_2_point_idx].point;

  // Here we define, which border would include this constraint
  const auto constraint_border_idx =
      (point - nearest_border_1_point).Length() <
              (nearest_border_2_point - point).Length()
          ? border_idx
          : overlapped_border_idx;

  constraints_.push_back(Constraint{
      .start_point_idx = nearest_border_1_point_idx,
      .end_point_idx = nearest_border_2_point_idx,
      .border_idx = border_idx,
  });
}

} // namespace diamond_fem::meshing
