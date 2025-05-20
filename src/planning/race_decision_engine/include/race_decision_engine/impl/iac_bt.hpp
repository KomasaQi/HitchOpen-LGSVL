// Copyright 2023 Siddharth Saha

#ifndef RACE_DECISION_ENGINE__IMPL__IAC_BT_HPP_
#define RACE_DECISION_ENGINE__IMPL__IAC_BT_HPP_

#include "race_decision_engine/base/base_bt_lifecycle_node.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace nodes
{
namespace implementations
{
class IacBt : public base::RaceDecisionEngineBtBaseLifecycleNode
{
public:
  explicit IacBt(const rclcpp::NodeOptions & options);

private:
  void Initialize() override;
  void RunTree() override;
  void PostRunTree() override;
};
}  // namespace implementations
}  // namespace nodes
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race

#endif  // RACE_DECISION_ENGINE__IMPL__IAC_BT_HPP_
