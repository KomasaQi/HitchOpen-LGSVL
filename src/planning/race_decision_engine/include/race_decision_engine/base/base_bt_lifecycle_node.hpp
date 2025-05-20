// Copyright 2023 Siddharth Saha

#ifndef RACE_DECISION_ENGINE__BASE__BASE_BT_LIFECYCLE_NODE_HPP_
#define RACE_DECISION_ENGINE__BASE__BASE_BT_LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/node_options.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

#include "race_decision_engine/base/base_lifecycle_node.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace nodes
{
namespace base
{
class RaceDecisionEngineBtBaseLifecycleNode : public RaceDecisionEngineBaseLifecycleNode
{
public:
  explicit RaceDecisionEngineBtBaseLifecycleNode(const rclcpp::NodeOptions & options);

protected:
  void Initialize() override;
  virtual void RunTree() = 0;
  virtual void PostRunTree();
  BT::Blackboard::Ptr & GetBlackboard();
  BT::Tree & GetBtTree();

private:
  template<typename T>
  void RegisterTreeNodeTree(const std::string & node_name);
  template<typename T>
  void RegisterTreeNodeLogClock(const std::string & node_name);
  template<typename T>
  void RegisterTreeNodeLogClockTree(const std::string & node_name);
  void RegisterTreeNodes();
  bool ProcessInputs(
    const common::RdeOutputs::SharedPtr & output_to_modify)
  override;

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree bt_tree_;

  std::unique_ptr<BT::PublisherZMQ> groot_monitor_;
  std::unique_ptr<BT::FileLogger> groot_log_file_;
};
}  // namespace base
}  // namespace nodes
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race

#endif  // RACE_DECISION_ENGINE__BASE__BASE_BT_LIFECYCLE_NODE_HPP_
