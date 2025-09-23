#include "geometry/line.hpp"
#include "geometry/point.hpp"
#include <meshing/point_stepper.hpp>

#include <algorithm>
#include <ranges>
#include <stdexcept>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <analysis_task/border.hpp>
#include <geometry/core.hpp>
#include <geometry/curve.hpp>
#include <geometry/interval.hpp>
#include <geometry/vec.hpp>

namespace diamond_fem::meshing {

namespace {

namespace bg = boost::geometry;

constexpr auto STABILITY_THRESHOLD = 0.02;
constexpr auto BORDER_WIDTH_ALLOWANCE = 0.1;
constexpr auto MIN_DISTANCE_COEFF_BETWEEN_INNER_AND_OUTER_POINTS = 0.3;

int CeilToInteger(double x) { return static_cast<int>(std::ceil(x)); }

internal::SteppingParameters
GetSteppingParameters1(const geometry::BoundingBox &bounding_box,
                       double triangle_side_length) {
  const auto line_count = CeilToInteger(
      (bounding_box.y_max - bounding_box.y_min + BORDER_WIDTH_ALLOWANCE) /
      triangle_side_length);

  return {
      .start_grid_point =
          geometry::Point(bounding_box.x_min, bounding_box.y_min),
      .direction = triangle_side_length * geometry::Vec(1, 0),
      .step_between_lines = triangle_side_length * geometry::Vec(0, 1),
      .line_count = line_count,
  };
}

internal::SteppingParameters
GetSteppingParameters2(const geometry::BoundingBox &bounding_box,
                       double triangle_side_length) {
  const auto bbox_width = bounding_box.x_max - bounding_box.x_min;
  const auto bbox_height = bounding_box.y_max - bounding_box.y_min;

  // Length of line connecting first and last grid points that passed as
  // start_grid_point. It is required to calculate line_count
  const auto line_length = bbox_width / std::cos(M_PI / 6) +
                           bbox_height / std::sin(M_PI / 6) +
                           BORDER_WIDTH_ALLOWANCE;

  return {
      .start_grid_point =
          geometry::Point(bounding_box.x_min, bounding_box.y_min),
      .direction = triangle_side_length *
                   geometry::Vec(std::sqrt(0.5), -std::sqrt(3) / 2),
      .step_between_lines = triangle_side_length *
                            geometry::Vec(std::sqrt(3) / 2, std::sqrt(0.5)),
      .line_count = CeilToInteger(line_length / triangle_side_length + 1),
  };
}

internal::SteppingParameters
GetSteppingParameters3(const geometry::BoundingBox &bounding_box,
                       double triangle_side_length) {
  const auto bbox_width = bounding_box.x_max - bounding_box.x_min;
  const auto bbox_height = bounding_box.y_max - bounding_box.y_min;

  // Same as in GetSteppingParameters2
  const auto line_length = bbox_width / std::cos(M_PI / 6) +
                           bbox_height / std::sin(M_PI / 6) +
                           BORDER_WIDTH_ALLOWANCE;

  return {
      .start_grid_point =
          geometry::Point(bounding_box.x_max, bounding_box.y_min),
      .direction = triangle_side_length *
                   geometry::Vec(std::sqrt(0.5), std::sqrt(3) / 2),
      .step_between_lines = triangle_side_length *
                            geometry::Vec(-std::sqrt(3) / 2, std::sqrt(0.5)),
      .line_count = CeilToInteger(line_length / triangle_side_length + 1),
  };
}

} // namespace

namespace internal {

std::vector<PointWithBorderInfo>
DeduplicatePoints(const std::vector<PointWithBorderInfo> &points) {
  using Point =
      boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
  using Value = std::pair<Point, PointWithBorderInfo>;

  auto rtree = bg::index::rtree<Value, bg::index::quadratic<16>>();
  auto result = std::vector<PointWithBorderInfo>();

  for (const auto &point : points) {
    auto bg_point = Point(point.point.GetX(), point.point.GetY());

    const auto search_box =
        bg::model::box<Point>(Point(point.point.GetX() - geometry::EPSILON,
                                    point.point.GetY() - geometry::EPSILON),
                              Point(point.point.GetX() + geometry::EPSILON,
                                    point.point.GetY() + geometry::EPSILON));

    auto is_duplicate = false;
    for (const auto &value : rtree | bg::index::adaptors::queried(
                                         bg::index::intersects(search_box))) {
      is_duplicate = true;
      break;
    }

    if (!is_duplicate) {
      rtree.insert(std::make_pair(bg_point, point));
      result.push_back(point);
    }
  }

  return result;
}

} // namespace internal

PointStepper::PointStepper(
    std::vector<std::shared_ptr<analysis_task::Border>> borders,
    double triangle_side_length)
    : borders_(borders), triangle_side_length_(triangle_side_length) {}

std::vector<PointWithBorderInfo> PointStepper::Step() {
  const auto borders_bounding_boxes =
      borders_ | std::views::transform([](const internal::BorderRef &border) {
        return border->curve->GetBoundingBox();
      }) |
      std::ranges::to<std::vector>();

  const auto bounding_box =
      geometry::CombineBoundingBoxes(borders_bounding_boxes);

  // First set of parallel lines goes through the bottom left corner of the
  // bounding box, lines are parallel with x axis.
  StepWithLines_(GetSteppingParameters1(bounding_box, triangle_side_length_));

  // Second set of parallel lines goes through the bottom left corner of the
  // bounding box, lines are 60 degrees about the first lines
  StepWithLines_(GetSteppingParameters2(bounding_box, triangle_side_length_));

  // Third set starts at the bottom right corner of the bounding box, lines are
  // 60 degrees about the first and second lines
  StepWithLines_(GetSteppingParameters3(bounding_box, triangle_side_length_));

  return internal::DeduplicatePoints(raw_points_);
}

void PointStepper::StepWithLines_(
    const internal::SteppingParameters &parameters) {
  for (int i = 0; i < parameters.line_count; i++) {
    const auto grid_point_on_line =
        parameters.start_grid_point +
        parameters.step_between_lines * static_cast<double>(i);

    StepWithOneLine_(grid_point_on_line, parameters.direction);
  }
}

void PointStepper::StepWithOneLine_(const geometry::Point &grid_point_on_line,
                                    const geometry::Vec &direction) {
  auto intersections_with_line = std::vector<PointWithBorderInfo>();

  const auto calculate_intersections_for_one_border =
      [&](const internal::BorderRef &border) {
        const auto intersections_with_one_border =
            border->curve->GetIntersectionsWithAxis(grid_point_on_line,
                                                    direction);

        // We dont want to see tangent points to circles, because they are
        // obstruct from interval retrieval
        const auto is_point_stable = [&](const geometry::Point &point) {
          const auto normal_at_point =
              border->curve->NormalAtPoint(point).Normalized();
          const auto cos_angle =
              std::abs(normal_at_point.DotProduct(direction.Normalized()));
          return cos_angle > STABILITY_THRESHOLD;
        };

        intersections_with_line.append_range(
            intersections_with_one_border |
            std::views::filter(is_point_stable) |
            std::views::transform([&](const auto &point) {
              return PointWithBorderInfo{point, border};
            }));
      };

  std::ranges::for_each(borders_, calculate_intersections_for_one_border);

  intersections_with_line =
      internal::DeduplicatePoints(intersections_with_line);

  // This poor, but working solution prevents us from case when line touches one
  // corner of our geometry. Solution is poor because if (line touches corner)
  // AND (line crosses geometry somewhere else), we couldn't determine what
  // intervals should we do from those three points
  if (intersections_with_line.size() == 1) {
    raw_points_.push_back(intersections_with_line[0]);
    return;
  }

  if (intersections_with_line.size() % 2) {
    throw std::runtime_error("intersections_with_line should be odd length. "
                             "Try change grid size a little bit");
  }

  std::ranges::sort(
      intersections_with_line, [](const auto &lhs, const auto &rhs) {
        return std::make_tuple(lhs.point.GetX(), lhs.point.GetY()) <
               std::make_tuple(rhs.point.GetX(), rhs.point.GetY());
      });

  for (int i = 0; i < intersections_with_line.size(); i += 2) {
    const auto interval_start_point = intersections_with_line[i];
    const auto interval_end_point = intersections_with_line[i + 1];

    const auto interval =
        geometry::Interval(interval_start_point.point, interval_end_point.point,
                           grid_point_on_line, direction);
    const auto stepped_interval = interval.DoStepping();

    // We don't want narrow triangles in our mesh, so we don't take points from
    // interval that are close to outer points
    const auto is_point_far_from_outer = [&](const auto &point) {
      const auto distance_to_start =
          (point - interval_start_point.point).Length();
      const auto distance_to_end = (point - interval_end_point.point).Length();
      return std::min(distance_to_start, distance_to_end) >
             triangle_side_length_ *
                 MIN_DISTANCE_COEFF_BETWEEN_INNER_AND_OUTER_POINTS;
    };

    const auto enrich_point_with_border_info = [&](const auto &point) {
      return PointWithBorderInfo{.point = point, .border = std::nullopt};
    };

    raw_points_.append_range(
        stepped_interval | std::views::filter(is_point_far_from_outer) |
        std::views::transform(enrich_point_with_border_info));

    raw_points_.push_back(interval_start_point);
    raw_points_.push_back(interval_end_point);
  }
}

} // namespace diamond_fem::meshing
