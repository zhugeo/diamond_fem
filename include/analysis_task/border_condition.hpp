#ifndef _DIAMOND_FEM_ANALYSIS_TASK_BORDER_CONDITION_HPP

namespace diamond_fem::analysis_task {

class BorderCondition {
public:
  // TODO
  virtual void _TODO() = 0;
};

class BorderConditionConstant : public BorderCondition {
public:
  BorderConditionConstant(double temperature);
  virtual void _TODO() override;

private:
  double temperature_;
};

class BorderConditionConstFlow : public BorderCondition {
public:
  BorderConditionConstFlow(double flow_outwards);
  virtual void _TODO() override;

private:
  double flow_outwards_;
};

class BorderConditionConvection : public BorderCondition {
public:
  BorderConditionConvection(double convection_coefficient);
  virtual void _TODO() override;

private:
  double convection_coefficient_;
};

} // namespace diamond_fem::analysis_task

#define _DIAMOND_FEM_ANALYSIS_TASK_BORDER_CONDITION_HPP
#endif // _DIAMOND_FEM_ANALYSIS_TASK_BORDER_CONDITION_HPP
