#include <analysis_task/border_condition.hpp>

namespace diamond_fem::analysis_task {

BorderConditionConstant::BorderConditionConstant(double temperature)
    : temperature_(temperature) {}

BorderConditionConstFlow::BorderConditionConstFlow(double flow_outwards)
    : flow_outwards_(flow_outwards) {}

BorderConditionConvection::BorderConditionConvection(
    double convection_coefficient)
    : convection_coefficient_(convection_coefficient) {}

void BorderConditionConstant::_TODO() {}
void BorderConditionConstFlow::_TODO() {}
void BorderConditionConvection::_TODO() {}

} // namespace diamond_fem::analysis_task
