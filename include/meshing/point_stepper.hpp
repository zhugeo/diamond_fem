#ifndef _DIAMOND_FEM_MESHING_POINT_STEPPER_HPP
#define _DIAMOND_FEM_MESHING_POINT_STEPPER_HPP

#include <optional>

#include <geometry/geometry_fwd.hpp>

#include <analysis_task/border.hpp>
#include <geometry/point.hpp>
#include <geometry/vec.hpp>

namespace diamond_fem::meshing {

namespace internal {

using BorderRef = std::shared_ptr<analysis_task::Border>;

// Parameters of stepping with a set of parallel lines
struct SteppingParameters {
  geometry::Point start_grid_point;
  geometry::Vec direction;
  geometry::Vec step_between_lines;
  int line_count;
};

} // namespace internal

struct PointWithBorderInfo {
  geometry::Point point;
  std::optional<internal::BorderRef> border;
};

// PointStepper converts borders of plate into set of points with border info.
// It is required to do triangulation further.
// Stepping is done as slicing plate with three sets of parallel lines, so
// internal shapes should be equilateral triangles
class PointStepper {
public:
  PointStepper(std::vector<std::shared_ptr<analysis_task::Border>> borders,
               double triangle_side_length);
  std::vector<PointWithBorderInfo> Step();

private:
  void StepWithLines_(const internal::SteppingParameters &parameters);
  void StepWithOneLine_(const geometry::Point &grid_point_on_line,
                        const geometry::Vec &direction);

  std::vector<internal::BorderRef> borders_;
  std::vector<PointWithBorderInfo> raw_points_; // Contains duplicates
  double triangle_side_length_;
};

namespace internal {

// DeduplicatePoints guarantees that if there are two "equal" points, there will
// be saved point that closer to start.
// Points p1 and p2 are "equal" if p2 is in square with side = 2*EPSILON and
// center in p1
std::vector<PointWithBorderInfo>
DeduplicatePoints(const std::vector<PointWithBorderInfo> &points);

class Interval {
public:
  Interval(const PointWithBorderInfo &start_point,
           const PointWithBorderInfo &end_point,
           const geometry::Point &point_on_grid_node,
           const geometry::Vec &direction_step);

  std::vector<PointWithBorderInfo> DoStepping() const;

private:
  PointWithBorderInfo start_point_;
  PointWithBorderInfo end_point_;
  geometry::Point point_on_grid_node_;
  geometry::Vec direction_step_;
};

} // namespace internal

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_POINT_STEPPER_HPP
