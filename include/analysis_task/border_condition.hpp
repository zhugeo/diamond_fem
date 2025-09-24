#ifndef _DIAMOND_FEM_ANALYSIS_TASK_BORDER_CONDITION_HPP
#define _DIAMOND_FEM_ANALYSIS_TASK_BORDER_CONDITION_HPP

namespace diamond_fem::analysis_task {

enum class BorderConditionType {
  ConstTemperature,
  ConstFlow,  // flow_outwards = value
  Convection, // flow_outwards = value * T
};

struct BorderCondition {
  BorderConditionType type;
  double value;
};

} // namespace diamond_fem::analysis_task

#endif // _DIAMOND_FEM_ANALYSIS_TASK_BORDER_CONDITION_HPP
