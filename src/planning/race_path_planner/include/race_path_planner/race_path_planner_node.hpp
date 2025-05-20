// Copyright 2022 Siddharth Saha
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

/// \copyright Copyright 2022 Siddharth Saha
/// \file
/// \brief This file defines the race_path_planner_node class.

#ifndef RACE_PATH_PLANNER__RACE_PATH_PLANNER_NODE_HPP_
#define RACE_PATH_PLANNER__RACE_PATH_PLANNER_NODE_HPP_

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "base_common/pubsub.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_msgs/msg/race_path_command.hpp"
#include "race_msgs/msg/rpp_telemetry.hpp"

#include "race_path_planner/frenet_path_planner.hpp"

namespace race
{
namespace race_path_planner
{

/// \class RacePathPlannerNode
/// \brief ROS 2 Node for Race Path Planner
class RacePathPlannerNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit RacePathPlannerNode(const rclcpp::NodeOptions & options);

  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & parameters);

private:
  bool has_seen_pose_{false};
  bool has_seen_objects_{false};
  bool has_seen_ttc_{false};
  bool use_perception_{false};
  bool verbose_{true};

  std::string frame_id_;
  std::vector<uint8_t> valid_idx_;

  double dt_{0.05};
  int debug_;

  rclcpp::TimerBase::SharedPtr step_timer_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
    m_trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    m_path_viz_publisher_;
  rclcpp::Publisher<race_msgs::msg::RacePathCommand>::SharedPtr
    m_rpp_command_publisher_;
  rclcpp::Publisher<race_msgs::msg::RppTelemetry>::SharedPtr
    m_rpp_telemetry_publisher_;

  pubsub::MsgSubscriber<race_msgs::msg::VehicleKinematicState>::UniquePtr m_pose_subscription_{};
  pubsub::MsgSubscriber<race_msgs::msg::TargetTrajectoryCommand>::UniquePtr
    m_ttc_subscription_{};
  pubsub::MsgSubscriber<autoware_auto_perception_msgs::msg::TrackedObjects>::UniquePtr
    m_objects_subscription_{};

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  std::unique_ptr<FrenetPathPlanner> m_frenet_path_planner_;
  FrenetPathParams m_frenet_params_;

  race_msgs::msg::RacePathCommand::SharedPtr tj_msg_;
  race_msgs::msg::RppTelemetry::SharedPtr telem_msg_;
  nav_msgs::msg::Path::SharedPtr path_msg_;
  visualization_msgs::msg::MarkerArray::SharedPtr marker_array_msg_;

  void declare_params();
  void initialize_params();
  void get_trajectory();
  void get_tree(const rclcpp::Time & step_time);
  void get_telemetry();
  void step();
};
}    // namespace race_path_planner
}  // namespace race

#endif  // RACE_PATH_PLANNER__RACE_PATH_PLANNER_NODE_HPP_
