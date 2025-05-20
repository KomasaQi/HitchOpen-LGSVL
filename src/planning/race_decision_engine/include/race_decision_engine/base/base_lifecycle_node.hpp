// Copyright 2023 Siddharth Saha

#ifndef RACE_DECISION_ENGINE__BASE__BASE_LIFECYCLE_NODE_HPP_
#define RACE_DECISION_ENGINE__BASE__BASE_LIFECYCLE_NODE_HPP_

#include <memory>
#include <cstdint>
#include <vector>
#include <string>

#include "ttl.hpp"
#include "ttl_tree.hpp"

#include "base_common/pubsub.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "race_decision_engine_parameters.hpp"

#include "race_decision_engine/common/input.hpp"
#include "race_decision_engine/common/output.hpp"

#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "race_msgs/msg/vehicle_manual_control_command.hpp"
#include "race_msgs/msg/vehicle_command.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "race_msgs/msg/fault_report.hpp"
#include "race_msgs/msg/push2_pass_report.hpp"

#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_msgs/msg/rde_telemetry.hpp"
#include "race_msgs/msg/tsp_telemetry.hpp"

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
class RaceDecisionEngineBaseLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RaceDecisionEngineBaseLifecycleNode(const rclcpp::NodeOptions & options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  const std::shared_ptr<ros_parameters::Params> GetParams();
  const std::shared_ptr<ros_parameters::Params>
  GetUpdatedParams();
  const common::RdeInputs::ConstSharedPtr GetInputs();
  const race::ttl::TtlTree::ConstSharedPtr GetTtlTree();
  const race::ttl::TtlIndex & GetLeftTtlIndex();
  const race::ttl::TtlIndex & GetRightTtlIndex();
  const race::ttl::TtlIndex & GetRaceTtlIndex();
  const race::ttl::TtlIndex & GetPitTtlIndex();
  const race::ttl::TtlIndex & GetOptimalTtlIndex();
  void UpdateParams();
  virtual bool ProcessInputs(
    const common::RdeOutputs::SharedPtr & output_to_modify);
  virtual void Initialize();

private:
  void Step();
  bool InputCheck();
  void PublishOutputs();
  bool IsTtlIndexValid(const int64_t ttl_index, const bool with_error);
  void UpdateTtlIndices(const bool with_error);


  bool InitParamListener();
  bool HaveSubscribersReceivedMessages();
  bool InitPubSub();
  bool ActivatePublishers();

  void InitializePublisher();
  void InitializeSubscriber();

  std::unique_ptr<ros_parameters::ParamListener>
  param_listener_{};
  rclcpp::TimerBase::SharedPtr timer_{};

  std::shared_ptr<ros_parameters::Params> rde_params_;
  common::RdeInputs::SharedPtr rde_inputs_{};
  common::RdeOutputs::SharedPtr rde_outputs_{};
  race::ttl::TtlTree::SharedPtr ttl_tree_{};

  pubsub::MsgSubscriber<race_msgs::msg::VehicleKinematicState,
    rclcpp_lifecycle::LifecycleNode>::UniquePtr localization_state_subscriber_{};
  pubsub::MsgSubscriber<race_msgs::msg::VehicleCommand,
    rclcpp_lifecycle::LifecycleNode>::UniquePtr rc_subscriber_{};
  pubsub::MsgSubscriber<race_msgs::msg::VehicleManualControlCommand,
    rclcpp_lifecycle::LifecycleNode>::UniquePtr input_manual_cmd_subscriber_{};
  pubsub::MsgSubscriber<autoware_auto_perception_msgs::msg::TrackedObjects,
    rclcpp_lifecycle::LifecycleNode>::UniquePtr opp_car_detections_subscriber_{};
  pubsub::MsgSubscriber<race_msgs::msg::FaultReport,
    rclcpp_lifecycle::LifecycleNode>::UniquePtr low_level_fault_report_subscriber_{};
  pubsub::MsgSubscriber<race_msgs::msg::Push2PassReport,
    rclcpp_lifecycle::LifecycleNode>::UniquePtr puhs2pass_report_subscriber_{};

  rclcpp_lifecycle::LifecyclePublisher<race_msgs::msg::TargetTrajectoryCommand>::SharedPtr
    ttc_publisher_{};
  rclcpp_lifecycle::LifecyclePublisher<race_msgs::msg::VehicleManualControlCommand>::SharedPtr
    output_manual_cmd_publisher_{};
  rclcpp_lifecycle::LifecyclePublisher<race_msgs::msg::RdeTelemetry>::SharedPtr rde_telem_publisher_
  {};

  bool lc_activated_{false};
  bool atleast_one_valid_localization_msg_{false};

  race::ttl::TtlIndex left_ttl_index_;
  race::ttl::TtlIndex right_ttl_index_;
  race::ttl::TtlIndex pit_ttl_index_;
  race::ttl::TtlIndex race_ttl_index_;
  race::ttl::TtlIndex optimal_ttl_index_;

  std::vector<std::string> tsp_name_segments_;
  std::vector<double> tsp_lap_percentages_;
  std::vector<double> tsp_speed_profiles_;

  std::vector<double> sf_lap_percentages_;
  std::vector<double> sf_scale_factors_;

  double prev_lap_time_{0.0};
};
}  // namespace base
}  // namespace nodes
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race

#endif  // RACE_DECISION_ENGINE__BASE__BASE_LIFECYCLE_NODE_HPP_
