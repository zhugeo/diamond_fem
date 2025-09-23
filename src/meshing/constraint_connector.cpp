#include <meshing/constraint_connector.hpp>

#include <stdexcept>

#include <fmt/format.h>

#include <geometry/core.hpp>
#include <geometry/curve.hpp>
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
  IndexateBorderStartPoints_();
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

void ConstraintConnector::IndexateBorderStartPoints_() {
  for (auto border_idx = 0; border_idx < borders_.size(); border_idx++) {
    const auto &curve = *(borders_[border_idx]->curve);
    const auto start_point = curve.GetParametricPoint(0);
    const auto boost_start_point = internal::BoostPoint{start_point.GetX(),
                                                        start_point.GetY()};
    border_start_points_.push_back(start_point);
    rtree_.insert(std::make_pair(boost_start_point, border_idx));
  }
}

void ConstraintConnector::ConnectAdjacentBorders_() {
  for (auto border_idx = 0; border_idx < borders_.size(); border_idx++) {
    const auto &curve = *(borders_[border_idx]->curve);
    const auto end_point = curve.GetParametricPoint(curve.Length());

    const auto search_box = internal::bg::model::box<internal::BoostPoint>{
        internal::BoostPoint{end_point.GetX() - geometry::EPSILON,
                             end_point.GetY() - geometry::EPSILON},
        internal::BoostPoint{end_point.GetX() + geometry::EPSILON,
                             end_point.GetY() + geometry::EPSILON},
    };
    auto overlapping_start_points =
        std::vector<internal::BoostPointWithBorderIndex>{};
    rtree_.query(internal::bgi::intersects(search_box),
                 std::back_inserter(overlapping_start_points));
    if (overlapping_start_points.size() != 1) {
      throw std::runtime_error(
          fmt::format("Found {} border start points overlapping with end point "
                      "for border {}",
                      overlapping_start_points.size(), curve.Description()));
    }
  }
}

} // namespace diamond_fem::meshing
