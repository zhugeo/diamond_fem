#ifndef _DIAMOND_FEM_GEOMETRY_SPATIAL_INDEX_HPP
#define _DIAMOND_FEM_GEOMETRY_SPATIAL_INDEX_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>

#include <geometry/core.hpp>
#include <geometry/point.hpp>

namespace diamond_fem::geometry {

namespace internal {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

} // namespace internal

template <typename T> class SpatialIndex {
public:
  using BoostPoint =
      internal::bg::model::point<double, 2, internal::bg::cs::cartesian>;
  using ValueType = std::pair<BoostPoint, T>;
  using RTree = internal::bgi::rtree<ValueType, internal::bgi::quadratic<16>>;

  SpatialIndex() = default;

  // Add a point with associated payload
  void AddPoint(const Point &point, T payload);

  // Remove a point from the index
  void RemovePoint(const Point &point, int limit = -1);

  // Find all points within a rectangle
  std::vector<std::pair<Point, T>>
  FindInRectangle(const BoundingBox &rectangle) const;

  // Find points within a radius from a center point
  std::vector<std::pair<Point, T>>
  FindInRadius(const Point &center, double radius, int limit = -1) const;

private:
  RTree rtree_;
  static BoostPoint ToBoostPoint(const Point &point);
};

// Implementation
template <typename T>
typename SpatialIndex<T>::BoostPoint
SpatialIndex<T>::ToBoostPoint(const Point &point) {
  return BoostPoint(point.GetX(), point.GetY());
}

template <typename T>
void SpatialIndex<T>::AddPoint(const Point &point, T payload) {
  rtree_.insert(std::make_pair(ToBoostPoint(point), std::move(payload)));
}

template <typename T>
void SpatialIndex<T>::RemovePoint(const Point &point, int limit) {
  BoostPoint boost_point = ToBoostPoint(point);
  // Remove entries with this point, up to the limit
  std::vector<ValueType> values_to_remove;
  // Use nearest with distance 0 to find exact matches
  rtree_.query(internal::bgi::nearest(boost_point, 1),
               std::back_inserter(values_to_remove));

  // Filter to only exact matches
  auto remove_it = std::ranges::remove_if(
      values_to_remove, [boost_point](const ValueType &value) {
        return internal::bg::distance(boost_point, value.first) > 0;
      });
  values_to_remove.erase(remove_it.begin(), values_to_remove.end());

  int removed_count = 0;
  for (const auto &value : values_to_remove) {
    if (limit >= 0 && removed_count >= limit) {
      break;
    }
    rtree_.remove(value);
    removed_count++;
  }
}

template <typename T>
std::vector<std::pair<Point, T>>
SpatialIndex<T>::FindInRectangle(const BoundingBox &rectangle) const {
  // Create a boost box from the bounding box
  BoostPoint min_point(rectangle.x_min, rectangle.y_min);
  BoostPoint max_point(rectangle.x_max, rectangle.y_max);
  auto box = internal::bg::model::box<BoostPoint>(min_point, max_point);

  std::vector<ValueType> result;
  rtree_.query(internal::bgi::intersects(box), std::back_inserter(result));

  // Convert from ValueType (pair of BoostPoint and T) to pair of Point and T
  std::vector<std::pair<Point, T>> converted_result;
  converted_result.reserve(result.size());
  for (const auto &value : result) {
    converted_result.emplace_back(
        Point(value.first.template get<0>(), value.first.template get<1>()),
        value.second);
  }
  return converted_result;
}

template <typename T>
std::vector<std::pair<Point, T>>
SpatialIndex<T>::FindInRadius(const Point &center, double radius,
                              int limit) const {
  // Create a circle for the query
  BoostPoint center_point = ToBoostPoint(center);
  auto circle = internal::bg::model::d2::point_xy<double>(
      center_point.template get<0>(), center_point.template get<1>());

  std::vector<ValueType> result;

  if (limit > 0) {
    // Limited query
    rtree_.query(internal::bgi::nearest(center_point, limit),
                 std::back_inserter(result));
    // Filter by radius
    result.erase(std::remove_if(result.begin(), result.end(),
                                [center_point, radius](const ValueType &value) {
                                  return internal::bg::distance(center_point,
                                                                value.first) >
                                         radius;
                                }),
                 result.end());
  } else {
    // Unlimited query within radius
    auto search_region = internal::bg::model::box<BoostPoint>(
        BoostPoint(center_point.template get<0>() - radius,
                   center_point.template get<1>() - radius),
        BoostPoint(center_point.template get<0>() + radius,
                   center_point.template get<1>() + radius));
    rtree_.query(internal::bgi::intersects(search_region),
                 std::back_inserter(result));
    // Filter by exact radius
    result.erase(std::remove_if(result.begin(), result.end(),
                                [center_point, radius](const ValueType &value) {
                                  return internal::bg::distance(center_point,
                                                                value.first) >
                                         radius;
                                }),
                 result.end());
  }

  // Convert from ValueType (pair of BoostPoint and T) to pair of Point and T
  std::vector<std::pair<Point, T>> converted_result;
  converted_result.reserve(result.size());
  for (const auto &value : result) {
    converted_result.emplace_back(
        Point(value.first.template get<0>(), value.first.template get<1>()),
        value.second);
  }
  return converted_result;
}

} // namespace diamond_fem::geometry

#endif // _DIAMOND_FEM_GEOMETRY_SPATIAL_INDEX_HPP
