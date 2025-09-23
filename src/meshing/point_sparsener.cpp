#include <iterator>
#include <meshing/point_sparsener.hpp>

#include <algorithm>
#include <limits>
#include <ranges>

#include <geometry/core.hpp>
#include <geometry/curve.hpp>
#include <geometry/point.hpp>

namespace diamond_fem::meshing {

namespace internal {

bool BoostPointsNear(const internal::BoostPoint &p1,
                     const internal::BoostPoint &p2) {
  return geometry::PointsNear(geometry::Point(p1.get<0>(), p1.get<1>()),
                              geometry::Point(p2.get<0>(), p2.get<1>()));
}

std::vector<PointWithBorderInfo>
SortPoints(std::vector<PointWithBorderInfo> points_to_sort) {
  std::ranges::sort(points_to_sort, [](const auto &lhs, const auto &rhs) {
    return std::make_tuple(lhs.point.GetX(), lhs.point.GetY()) <
           std::make_tuple(rhs.point.GetX(), rhs.point.GetY());
  });

  return std::move(points_to_sort);
}

} // namespace internal

PointSparsener::PointSparsener(const SparsingParameters &parameters,
                               const std::vector<internal::BorderRef> &borders,
                               std::vector<PointWithBorderInfo> points)
    : parameters_(parameters) {

  const auto calculate_distance_to_border = [&](const auto &point) {
    auto distance = std::numeric_limits<double>::max();
    std::ranges::for_each(borders, [&](const auto &border) {
      distance = std::min(distance, border->curve->DistanceToPoint(point));
    });
    return distance;
  };

  const auto sorted_points = internal::SortPoints(std::move(points));

  points_ = sorted_points |
            std::views::transform([&](const auto &point_with_border_info) {
              return internal::PointWrapper{
                  .point_with_border_info = point_with_border_info,
                  .boost_point =
                      internal::BoostPoint(point_with_border_info.point.GetX(),
                                           point_with_border_info.point.GetY()),
                  .distance_to_border = calculate_distance_to_border(
                      point_with_border_info.point),
              };
            }) |
            std::ranges::to<std::vector>();

  RebuildRTree_();
}

std::vector<PointWithBorderInfo> PointSparsener::Sparse() {
  for (const auto &pass_parameters : parameters_.sparsing_passes) {
    DoSparsingPass_(pass_parameters);
  }
  return std::move(points_) | std::views::transform([](auto wrapped_point) {
           return std::move(wrapped_point.point_with_border_info);
         }) |
         std::ranges::to<std::vector>();
}

void PointSparsener::RebuildRTree_() {
  std::vector<internal::BoostPoint> values;
  values.reserve(points_.size());

  for (size_t i = 0; i < points_.size(); ++i) {
    const auto &point = points_[i].boost_point;
    values.emplace_back(point);
  }

  rtree_ = internal::RTree(values);
}

void PointSparsener::DoSparsingPass_(
    const SparsingPassParameters &pass_parameters) {
  auto points_to_remove = std::vector<int>();

  for (int i = 0; i < points_.size(); ++i) {
    const auto &point = points_[i];
    if (ShouldRemovePoint_(point, pass_parameters)) {
      points_to_remove.push_back(i);
      rtree_.remove(point.boost_point);
    }
  }

  auto filtered_points = std::vector<internal::PointWrapper>();
  auto j = 0; // index in points_to_remove vector
  for (int i = 0; i < points_.size(); i++) {
    if (j <= points_to_remove.size() && i == points_to_remove[j]) {
      j++;
      continue;
    }
    filtered_points.emplace_back(std::move(points_[i]));
  }

  points_ = std::move(filtered_points);
}

bool PointSparsener::ShouldRemovePoint_(
    const internal::PointWrapper &point,
    const SparsingPassParameters &pass_parameters) {

  if (point.point_with_border_info.border.has_value()) {
    return false;
  }

  if (point.distance_to_border < pass_parameters.min_distance_to_border) {
    return false;
  }

  // We request one point more, because one of them should be template point
  // itself
  const auto neighbor_count = pass_parameters.num_neighbors + 1;

  auto nearest_points = std::vector<internal::BoostPoint>{};
  rtree_.query(internal::bgi::nearest(point.boost_point, neighbor_count),
               std::back_inserter(nearest_points));

  for (const auto &nearest_point : nearest_points) {
    // Skip the point itself
    if (internal::BoostPointsNear(nearest_point, point.boost_point)) {
      continue;
    }

    const auto distance =
        internal::bg::distance(point.boost_point, nearest_point);
    if (distance > pass_parameters.max_distance_to_neighbor_point) {
      return false;
    }
  }

  return true;
}

} // namespace diamond_fem::meshing
