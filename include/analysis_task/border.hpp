#ifndef _DIAMOND_FEM_ANALYSIS_TASK_BORDER_HPP
#define _DIAMOND_FEM_ANALYSIS_TASK_BORDER_HPP

#include <memory>

#include <analysis_task/border_condition.hpp>
#include <geometry/geometry_fwd.hpp>

namespace diamond_fem::analysis_task {

struct Border {
  const BorderCondition border_condition;
  const std::shared_ptr<geometry::Curve> curve;
};

} // namespace diamond_fem::analysis_task

#endif // _DIAMOND_FEM_ANALYSIS_TASK_BORDER_HPP
