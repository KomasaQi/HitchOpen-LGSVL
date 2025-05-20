// Copyright 2023 Siddharth Saha

#include "race_decision_engine/base/base_lifecycle_node.hpp"

#include <stdexcept>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "race_decision_engine/common/utils.hpp"

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
RaceDecisionEngineBaseLifecycleNode::RaceDecisionEngineBaseLifecycleNode(
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("race_decision_engine_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Creating base lifecycle node");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaceDecisionEngineBaseLifecycleNode::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Configuring base lifecycle node");
  (void)previous_state;

  if (!InitParamListener()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  rde_params_ = std::make_shared<ros_parameters::Params>();
  *rde_params_ = param_listener_->get_params();
  ttl_tree_ = std::make_shared<race::ttl::TtlTree>(rde_params_->ttl_csv_dir.c_str());
  UpdateTtlIndices(true);

  if (!InitPubSub()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  timer_ =
    rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(
      rde_params_->step_time_period_sec), std::bind(
      &RaceDecisionEngineBaseLifecycleNode::Step,
      this));

  rde_inputs_ = std::make_shared<common::RdeInputs>();
  rde_inputs_->input_manual_cmd_msg = nullptr;
  rde_inputs_->localization_state_msg = nullptr;
  rde_inputs_->opp_car_detections_msg = nullptr;
  rde_inputs_->race_control_msg = nullptr;
  rde_inputs_->low_level_fault_report_msg = nullptr;
  rde_inputs_->push2pass_report_msg = nullptr;
  rde_outputs_ = std::make_shared<common::RdeOutputs>();
  rde_outputs_->ttc_msg = nullptr;
  rde_outputs_->output_manual_cmd_msg = nullptr;

  Initialize();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaceDecisionEngineBaseLifecycleNode::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Activating base lifecycle node");
  (void)previous_state;

  if (!ActivatePublishers()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  timer_->reset();

  lc_activated_ = true;
  prev_lap_time_ = now().seconds();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaceDecisionEngineBaseLifecycleNode::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating base lifecycle node");
  (void)previous_state;

  timer_->cancel();
  pubsub::deactivate_publisher(ttc_publisher_);
  pubsub::deactivate_publisher(output_manual_cmd_publisher_);
  pubsub::deactivate_publisher(rde_telem_publisher_);

  lc_activated_ = false;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaceDecisionEngineBaseLifecycleNode::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down base lifecycle node");
  (void)previous_state;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaceDecisionEngineBaseLifecycleNode::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up base lifecycle from error state");
  (void)previous_state;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RaceDecisionEngineBaseLifecycleNode::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Base Lifecycle Node hit an error");
  (void)previous_state;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

const std::shared_ptr<ros_parameters::Params>
RaceDecisionEngineBaseLifecycleNode::GetParams()
{
  return rde_params_;
}

const std::shared_ptr<ros_parameters::Params>
RaceDecisionEngineBaseLifecycleNode::GetUpdatedParams()
{
  UpdateParams();
  return GetParams();
}

const common::RdeInputs::ConstSharedPtr
RaceDecisionEngineBaseLifecycleNode::GetInputs()
{
  return rde_inputs_;
}

const race::ttl::TtlTree::ConstSharedPtr RaceDecisionEngineBaseLifecycleNode::GetTtlTree()
{
  return ttl_tree_;
}

const race::ttl::TtlIndex & RaceDecisionEngineBaseLifecycleNode::GetLeftTtlIndex()
{
  return left_ttl_index_;
}

const race::ttl::TtlIndex & RaceDecisionEngineBaseLifecycleNode::GetRightTtlIndex()
{
  return right_ttl_index_;
}

const race::ttl::TtlIndex & RaceDecisionEngineBaseLifecycleNode::GetRaceTtlIndex()
{
  return race_ttl_index_;
}

const race::ttl::TtlIndex & RaceDecisionEngineBaseLifecycleNode::GetPitTtlIndex()
{
  return pit_ttl_index_;
}

const race::ttl::TtlIndex & RaceDecisionEngineBaseLifecycleNode::GetOptimalTtlIndex()
{
  return optimal_ttl_index_;
}

void RaceDecisionEngineBaseLifecycleNode::UpdateParams()
{
  if (param_listener_->is_old(*rde_params_)) {
    *rde_params_ = param_listener_->get_params();
  }
}

bool RaceDecisionEngineBaseLifecycleNode::ProcessInputs(
  const common::RdeOutputs::SharedPtr & output_to_modify)
{
  RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
    "ProcessInputs from BLN doing nothing");
  (void) output_to_modify;
  return false;
}

void RaceDecisionEngineBaseLifecycleNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initialize from BLN doing nothing");
}

void RaceDecisionEngineBaseLifecycleNode::Step()
{
  auto start_time = this->now();
  UpdateParams();
  UpdateTtlIndices(false);
  if (!InputCheck()) {
    return;
  }
  rde_inputs_->input_manual_cmd_msg = input_manual_cmd_subscriber_->last_received_msg();
  rde_inputs_->input_manual_cmd_diff =
    (start_time - input_manual_cmd_subscriber_->latest_msg_time()).seconds();
  rde_inputs_->race_control_msg = rc_subscriber_->last_received_msg();
  rde_inputs_->race_control_diff = (start_time - rc_subscriber_->latest_msg_time()).seconds();
  rde_inputs_->localization_state_msg = localization_state_subscriber_->last_received_msg();
  rde_inputs_->localization_diff =
    (start_time - localization_state_subscriber_->latest_msg_time()).seconds();
  rde_inputs_->low_level_fault_report_msg = low_level_fault_report_subscriber_->last_received_msg();
  rde_inputs_->low_level_fault_report_diff =
    (start_time - low_level_fault_report_subscriber_->latest_msg_time()).seconds();
  if (rde_params_->use_perception) {
    rde_inputs_->opp_car_detections_msg = opp_car_detections_subscriber_->last_received_msg();
    rde_inputs_->opp_car_detections_diff =
      (start_time - opp_car_detections_subscriber_->latest_msg_time()).seconds();
  }
  rde_inputs_->push2pass_report_msg = puhs2pass_report_subscriber_->last_received_msg();
  rde_inputs_->push2pass_report_diff =
    (start_time - puhs2pass_report_subscriber_->latest_msg_time()).seconds();
  if (!lc_activated_) {
    return;
  }
  if (!rde_outputs_->ttc_msg) {
    rde_outputs_->ttc_msg = std::make_shared<race_msgs::msg::TargetTrajectoryCommand>();
  }
  if (!rde_outputs_->output_manual_cmd_msg) {
    rde_outputs_->output_manual_cmd_msg =
      std::make_shared<race_msgs::msg::VehicleManualControlCommand>();
  }
  if (!rde_outputs_->rde_telem_msg) {
    rde_outputs_->rde_telem_msg = std::make_shared<race_msgs::msg::RdeTelemetry>();
  }
  rde_outputs_->ttc_msg->stamp = start_time;
  rde_outputs_->rde_telem_msg->stamp = start_time;
  *rde_outputs_->output_manual_cmd_msg = *rde_inputs_->input_manual_cmd_msg;
  double prev_lap_percentage = rde_outputs_->rde_telem_msg->lap_percentage;
  if (!ProcessInputs(rde_outputs_)) {
    rde_outputs_->ttc_msg->target_speed = 0.0;
    rde_outputs_->ttc_msg->stop_type.stop_type = race_msgs::msg::StopType::STOP_TYPE_EMERGENCY;
    rde_outputs_->ttc_msg->strategy_type.strategy_type =
      race_msgs::msg::StrategyType::CRUISE_CONTROL;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(),
      GetParams()->non_critical_log_time_period_ms,
      "Process Inputs returned false. Overriding to emergency");
  }
  rde_outputs_->ttc_msg->target_speed = rde_outputs_->ttc_msg->target_speed * common::MPH_TO_MPS;
  if (rde_outputs_->ttc_msg->target_speed == 0.0 &&
    rde_outputs_->ttc_msg->stop_type.stop_type ==
    race_msgs::msg::StopType::STOP_TYPE_NOMINAL)
  {
    rde_outputs_->ttc_msg->stop_type.stop_type = race_msgs::msg::StopType::STOP_TYPE_SAFE;
  }
  if (rde_outputs_->ttc_msg->target_gap == 0.0) {
    rde_outputs_->ttc_msg->strategy_type.strategy_type =
      race_msgs::msg::StrategyType::CRUISE_CONTROL;
  }
  rde_outputs_->rde_telem_msg->trajectory_command = *rde_outputs_->ttc_msg;
  if (prev_lap_percentage > rde_outputs_->rde_telem_msg->lap_percentage) {
    prev_lap_time_ = now().seconds();
  }
  rde_outputs_->rde_telem_msg->lap_time = now().seconds() - prev_lap_time_;
  rde_outputs_->rde_telem_msg->tsp_telemetry.tsp_name = GetParams()->target_speed_profiles.tsp_name;
  rde_outputs_->rde_telem_msg->tsp_telemetry.enabled = GetParams()->target_speed_profiles.enabled;
  rde_outputs_->rde_telem_msg->tsp_telemetry.tsp_segment_names =
    GetParams()->target_speed_profiles.tsp_segment_names;
  rde_outputs_->rde_telem_msg->tsp_telemetry.percentage_points =
    GetParams()->target_speed_profiles.lap_percentages;
  rde_outputs_->rde_telem_msg->tsp_telemetry.speeds =
    GetParams()->target_speed_profiles.speed_profiles;
  rde_outputs_->rde_telem_msg->rc_diff = rde_inputs_->race_control_diff;
  rde_outputs_->rde_telem_msg->joystick_diff = rde_inputs_->input_manual_cmd_diff;
  rde_outputs_->rde_telem_msg->pose_diff = rde_inputs_->localization_diff;
  rde_outputs_->rde_telem_msg->low_level_fault_diff =
    rde_inputs_->low_level_fault_report_diff;
  rde_outputs_->rde_telem_msg->opp_detections_diff = rde_inputs_->opp_car_detections_diff;
  PublishOutputs();
}

bool RaceDecisionEngineBaseLifecycleNode::InputCheck()
{
  if (!HaveSubscribersReceivedMessages()) {
    return false;
  } else {
    if (!atleast_one_valid_localization_msg_) {
      if (localization_state_subscriber_->last_received_msg()->source_status_code == 0) {
        atleast_one_valid_localization_msg_ = true;
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
          "Localization state not valid");
        return false;
      }
    }
  }
  return true;
}

void RaceDecisionEngineBaseLifecycleNode::PublishOutputs()
{
  if (rde_outputs_->ttc_msg) {
    ttc_publisher_->publish(*rde_outputs_->ttc_msg);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
      "No Target Trajectory Command values filled");
  }
  if (rde_outputs_->output_manual_cmd_msg) {
    output_manual_cmd_publisher_->publish(*rde_outputs_->output_manual_cmd_msg);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
      "No Output Manual Command values filled");
  }
  if (rde_outputs_->rde_telem_msg) {
    rde_telem_publisher_->publish(*rde_outputs_->rde_telem_msg);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
      "No RDE Telemetry values filled");
  }
}


bool RaceDecisionEngineBaseLifecycleNode::IsTtlIndexValid(
  const int64_t ttl_index,
  const bool with_error)
{
  if (!ttl_tree_->is_ttl_index_valid(ttl_index)) {
    RCLCPP_ERROR(this->get_logger(), "TTL index %ld is not valid", ttl_index);
    if (with_error) {
      throw std::runtime_error("TTL index " + std::to_string(ttl_index) + " is not valid");
    }
    return false;
  }
  return true;
}

void RaceDecisionEngineBaseLifecycleNode::UpdateTtlIndices(const bool with_error)
{
  if (IsTtlIndexValid(rde_params_->ttls.left_ttl_index, with_error)) {
    left_ttl_index_ = static_cast<race::ttl::TtlIndex>(rde_params_->ttls.left_ttl_index);
  }
  if (IsTtlIndexValid(rde_params_->ttls.right_ttl_index, with_error)) {
    right_ttl_index_ = static_cast<race::ttl::TtlIndex>(rde_params_->ttls.right_ttl_index);
  }
  if (IsTtlIndexValid(rde_params_->ttls.race_ttl_index, with_error)) {
    race_ttl_index_ = static_cast<race::ttl::TtlIndex>(rde_params_->ttls.race_ttl_index);
  }
  if (IsTtlIndexValid(rde_params_->ttls.pit_ttl_index, with_error)) {
    pit_ttl_index_ = static_cast<race::ttl::TtlIndex>(rde_params_->ttls.pit_ttl_index);
  }
  if (IsTtlIndexValid(rde_params_->ttls.optimal_ttl_index, with_error)) {
    optimal_ttl_index_ = static_cast<race::ttl::TtlIndex>(rde_params_->ttls.optimal_ttl_index);
  }
}

bool RaceDecisionEngineBaseLifecycleNode::InitParamListener()
{
  try {
    param_listener_ = std::make_unique<ros_parameters::ParamListener>(
      this->get_node_parameters_interface());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in ParamListener: %s", e.what());
    return false;
  }
}

bool RaceDecisionEngineBaseLifecycleNode::HaveSubscribersReceivedMessages()
{
  bool has_seen_all_messages = true;
  if (!localization_state_subscriber_->has_seen_msg()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
      "Localization state not received");
    has_seen_all_messages = false;
  }
  if (!low_level_fault_report_subscriber_->has_seen_msg()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
      "Low Level Fault Report not received");
    has_seen_all_messages = false;
  }
  if (!rc_subscriber_->has_seen_msg()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
      "Race Control not received");
    has_seen_all_messages = false;
  }
  if (!input_manual_cmd_subscriber_->has_seen_msg()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
      "Manual Command not received");
    has_seen_all_messages = false;
  }
  if (!opp_car_detections_subscriber_->has_seen_msg() && rde_params_->use_perception) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
      "Opponent Car Detections not received. Use perception is set to %d",
      rde_params_->use_perception);
    has_seen_all_messages = false;
  }
  if (!puhs2pass_report_subscriber_->has_seen_msg()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), rde_params_->non_critical_log_time_period_ms,
      "Push2Pass Report not received");
    has_seen_all_messages = false;
  }
  return has_seen_all_messages;
}

bool RaceDecisionEngineBaseLifecycleNode::InitPubSub()
{
  InitializePublisher();
  InitializeSubscriber();
  return true;
}

bool RaceDecisionEngineBaseLifecycleNode::ActivatePublishers()
{
  pubsub::activate_publisher(ttc_publisher_);
  pubsub::activate_publisher(output_manual_cmd_publisher_);
  pubsub::activate_publisher(rde_telem_publisher_);

  return true;
}

void RaceDecisionEngineBaseLifecycleNode::InitializePublisher()
{
  pubsub::publish_to(this, ttc_publisher_, "ttc");
  pubsub::publish_to(this, output_manual_cmd_publisher_, "manual_cmd_filtered");
  pubsub::publish_to(this, rde_telem_publisher_, "rde_telemetry");
}

void RaceDecisionEngineBaseLifecycleNode::InitializeSubscriber()
{
  pubsub::subscribe_from(this, localization_state_subscriber_, "localization_state");
  pubsub::subscribe_from(this, opp_car_detections_subscriber_, "opp_cars");
  pubsub::subscribe_from(this, input_manual_cmd_subscriber_, "manual_cmd_raw");
  pubsub::subscribe_from(this, rc_subscriber_, "race_control");
  pubsub::subscribe_from(this, low_level_fault_report_subscriber_, "low_level_fault_report");
  pubsub::subscribe_from(this, puhs2pass_report_subscriber_, "push2pass_report");
}
}  // namespace base
}  // namespace nodes
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
