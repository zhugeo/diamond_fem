#ifndef _DIAMOND_FEM_ANALYSIS_TASK_BORDER_HPP
#define _DIAMOND_FEM_ANALYSIS_TASK_BORDER_HPP

#include <memory>
#include <vector>

#include <analysis_task/border_condition.hpp>
#include <geometry/geometry_fwd.hpp>

namespace diamond_fem::analysis_task {

struct Border {
  std::shared_ptr<const BorderCondition> border_condition;
  std::shared_ptr<const geometry::Curve> curve;
};

} // namespace diamond_fem::analysis_task

#endif // _DIAMOND_FEM_ANALYSIS_TASK_BORDER_HPP
