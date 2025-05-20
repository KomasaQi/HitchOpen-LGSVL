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

#include <vector>
#include <string>
#include <memory>
#include <limits>

#include "race_path_planner/race_path_planner_node.hpp"

namespace race
{
namespace race_path_planner
{

RacePathPlannerNode::RacePathPlannerNode(const rclcpp::NodeOptions & options)
:  rclcpp::Node("race_path_planner_node", options)
{
  telem_msg_ = std::make_shared<race_msgs::msg::RppTelemetry>();
  marker_array_msg_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
  path_msg_ = std::make_shared<nav_msgs::msg::Path>();
  tj_msg_ = std::make_shared<race_msgs::msg::RacePathCommand>();
  declare_params();
  initialize_params();
  m_frenet_path_planner_ = std::make_unique<FrenetPathPlanner>(m_frenet_params_);
  pubsub::publish_to(this, m_trajectory_publisher_, "local_trajectory");
  pubsub::publish_to(this, m_path_viz_publisher_, "trajectory_viz");
  pubsub::publish_to(this, m_rpp_command_publisher_, "rpp_command");
  pubsub::publish_to(this, m_rpp_telemetry_publisher_, "rpp_telemetry");

  pubsub::subscribe_from(this, m_pose_subscription_, "state");
  pubsub::subscribe_from(this, m_ttc_subscription_, "trajectory_command");
  pubsub::subscribe_from(this, m_objects_subscription_, "detected_objects");

  step_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(this->get_parameter("dt").as_double()), [this] {
      step();
    });

  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&RacePathPlannerNode::param_callback, this, std::placeholders::_1));
}

void RacePathPlannerNode::declare_params()
{
  this->declare_parameter<std::string>("ttl_directory");
  this->declare_parameter<double>("dt");
  this->declare_parameter<double>("ds");
  this->declare_parameter<bool>("use_perception");
  this->declare_parameter<int>("debug");
  this->declare_parameter<int>("n_threads");
  this->declare_parameter<double>("max_left_deviation");
  this->declare_parameter<double>("max_right_deviation");
  this->declare_parameter<double>("road_inc");
  this->declare_parameter<double>("max_speed_low_deviation");
  this->declare_parameter<double>("max_speed_high_deviation");
  this->declare_parameter<double>("speed_inc");
  this->declare_parameter<double>("max_lat_vel_low_deviation");
  this->declare_parameter<double>("max_lat_vel_high_deviation");
  this->declare_parameter<double>("lat_vel_inc");
  this->declare_parameter<double>("t_period");
  this->declare_parameter<double>("frenet_dt");
  this->declare_parameter<double>("obj_safety_thresh");
  this->declare_parameter<double>("boundary_safety_thresh");
  this->declare_parameter<double>("detection_radius");
  this->declare_parameter<double>("follow_desired_gap");
  this->declare_parameter<double>("kp_gap");
  this->declare_parameter<double>("kc_prev");
  this->declare_parameter<double>("kj");
  this->declare_parameter<double>("kd");
  this->declare_parameter<double>("klat");
  this->declare_parameter<double>("klon");
  this->declare_parameter<double>("max_acc");
  this->declare_parameter<double>("max_curv");
  this->declare_parameter<double>("marker_size");
  this->declare_parameter<double>("marker_lifetime");
  this->declare_parameter<double>("max_yaw_diff");
  this->declare_parameter<double>("min_speed");
  this->declare_parameter<std::string>("frame_id");
  this->declare_parameter<int>("center_ttl");
}

void RacePathPlannerNode::initialize_params()
{
  m_frenet_params_.ttl_dir = this->get_parameter("ttl_directory").as_string();
  m_frenet_params_.ds = this->get_parameter("ds").as_double();
  use_perception_ = this->get_parameter("use_perception").as_bool();
  debug_ = this->get_parameter("debug").as_int();
  frame_id_ = this->get_parameter("frame_id").as_string();
  m_frenet_params_.n_threads = this->get_parameter("n_threads").as_int();
  m_frenet_params_.max_left_deviation = this->get_parameter("max_left_deviation").as_double();
  m_frenet_params_.max_right_deviation = this->get_parameter("max_right_deviation").as_double();
  m_frenet_params_.road_inc = this->get_parameter("road_inc").as_double();
  m_frenet_params_.max_speed_low_deviation =
    this->get_parameter("max_speed_low_deviation").as_double();
  m_frenet_params_.max_speed_high_deviation =
    this->get_parameter("max_speed_high_deviation").as_double();
  m_frenet_params_.speed_inc = this->get_parameter("speed_inc").as_double();
  m_frenet_params_.max_lat_vel_low_deviation =
    this->get_parameter("max_lat_vel_low_deviation").as_double();
  m_frenet_params_.max_lat_vel_high_deviation =
    this->get_parameter("max_lat_vel_high_deviation").as_double();
  m_frenet_params_.lat_vel_inc = this->get_parameter("lat_vel_inc").as_double();
  m_frenet_params_.t_period = this->get_parameter("t_period").as_double();
  m_frenet_params_.dt = this->get_parameter("frenet_dt").as_double();
  m_frenet_params_.obj_safety_thresh = this->get_parameter("obj_safety_thresh").as_double();
  m_frenet_params_.boundary_safety_thresh =
    this->get_parameter("boundary_safety_thresh").as_double();
  m_frenet_params_.detection_radius = this->get_parameter("detection_radius").as_double();
  m_frenet_params_.follow_desired_gap = this->get_parameter("follow_desired_gap").as_double();
  m_frenet_params_.kp_gap = this->get_parameter("kp_gap").as_double();
  m_frenet_params_.kc_prev = this->get_parameter("kc_prev").as_double();
  m_frenet_params_.kj = this->get_parameter("kj").as_double();
  m_frenet_params_.kd = this->get_parameter("kd").as_double();
  m_frenet_params_.klat = this->get_parameter("klat").as_double();
  m_frenet_params_.klon = this->get_parameter("klon").as_double();
  m_frenet_params_.max_acc = this->get_parameter("max_acc").as_double();
  m_frenet_params_.max_curv = this->get_parameter("max_curv").as_double();
  m_frenet_params_.marker_size = this->get_parameter("marker_size").as_double();
  m_frenet_params_.marker_lifetime = this->get_parameter("marker_lifetime").as_double();
  m_frenet_params_.max_yaw_diff = this->get_parameter("max_yaw_diff").as_double();
  m_frenet_params_.min_speed = this->get_parameter("min_speed").as_double();
  m_frenet_params_.center_ttl = this->get_parameter("center_ttl").as_int();
}

rcl_interfaces::msg::SetParametersResult RacePathPlannerNode::param_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const auto & param : parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (param.get_name() == "use_perception") {
        use_perception_ = param.as_bool();
        result.successful = true;
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (param.as_int() > 1) {
        if (param.get_name() == "n_threads") {
          m_frenet_params_.n_threads = param.as_int();
          result.successful = true;
        }
      } else if (param.as_int() > 0) {
        if (param.get_name() == "debug") {
          debug_ = param.as_int();
          result.successful = true;
        } else if (param.get_name() == "center_ttl") {
          if (m_frenet_path_planner_->validate_ttl_index(param.as_int())) {
            m_frenet_params_.center_ttl = static_cast<uint8_t>(param.as_int());
          }
        }
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (param.as_double() >= 0.0) {
        if (param.get_name() == "max_left_deviation") {
          m_frenet_params_.max_left_deviation = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "max_right_deviation") {
          m_frenet_params_.max_right_deviation = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "max_speed_low_deviation") {
          m_frenet_params_.max_speed_low_deviation = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "max_speed_high_deviation") {
          m_frenet_params_.max_speed_high_deviation = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "max_lat_vel_low_deviation") {
          m_frenet_params_.max_lat_vel_low_deviation = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "max_lat_vel_high_deviation") {
          m_frenet_params_.max_lat_vel_high_deviation = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "detection_radius") {
          m_frenet_params_.detection_radius = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "kc_prev") {
          m_frenet_params_.kc_prev = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "kj") {
          m_frenet_params_.kj = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "kd") {
          m_frenet_params_.kd = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "klat") {
          m_frenet_params_.klat = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "klon") {
          m_frenet_params_.klon = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "max_acc") {
          m_frenet_params_.max_acc = param.as_double();
          result.successful = true;
        } else if (param.get_name() == "max_curv") {
          m_frenet_params_.max_curv = param.as_double();
          result.successful = true;
        } else if (param.as_double() > 0.0) {
          if (param.get_name() == "road_inc") {
            m_frenet_params_.road_inc = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "speed_inc") {
            m_frenet_params_.speed_inc = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "speed_inc") {
            m_frenet_params_.speed_inc = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "lat_vel_inc") {
            m_frenet_params_.lat_vel_inc = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "t_period") {
            m_frenet_params_.t_period = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "frenet_dt") {
            m_frenet_params_.dt = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "marker_size") {
            m_frenet_params_.marker_size = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "marker_lifetime") {
            m_frenet_params_.marker_lifetime = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "follow_desired_gap") {
            m_frenet_params_.follow_desired_gap = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "kp_gap") {
            m_frenet_params_.kp_gap = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "max_yaw_diff") {
            m_frenet_params_.max_yaw_diff = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "min_speed") {
            m_frenet_params_.min_speed = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "boundary_safety_thresh") {
            m_frenet_params_.boundary_safety_thresh = param.as_double();
            result.successful = true;
          } else if (param.get_name() == "obj_safety_thresh") {
            m_frenet_params_.obj_safety_thresh = param.as_double();
            result.successful = true;
          }
        }
      }
    }
  }
  if (result.successful) {
    m_frenet_path_planner_->update_params(m_frenet_params_);
  }
  result.reason = result.successful ? "success" : "failure";
  return result;
}

void RacePathPlannerNode::step()
{
  has_seen_pose_ = m_pose_subscription_->has_seen_msg();
  has_seen_objects_ = m_objects_subscription_->has_seen_msg();
  has_seen_ttc_ = m_ttc_subscription_->has_seen_msg();

  if (!has_seen_pose_ || (!has_seen_objects_ && use_perception_) || !has_seen_ttc_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      debug_,
      "Signal not received.  got pose: %d, got objects: %d, got ttc: %d",
      has_seen_pose_, has_seen_objects_, has_seen_ttc_);
    return;
  }

  auto step_time = this->now();
  auto pose_diff = (step_time - m_pose_subscription_->latest_msg_time()).seconds();
  double objects_diff = 0.0;
  if (use_perception_) {
    objects_diff =
      (step_time - m_objects_subscription_->latest_msg_time()).seconds();
  }
  auto ttc_diff = (step_time - m_ttc_subscription_->latest_msg_time()).seconds();
  auto pose_msg = m_pose_subscription_->take();
  if (use_perception_) {
    auto objects_msg = m_objects_subscription_->take();
    if (objects_msg) {
      m_frenet_path_planner_->update_objects(objects_msg);
    }
  }
  auto ttc_msg = m_ttc_subscription_->take();
  if (pose_msg) {
    m_frenet_path_planner_->update_pose(pose_msg);
  }
  if (ttc_msg) {
    m_frenet_path_planner_->update_ttc(ttc_msg);
  }
  tj_msg_->stamp = step_time;
  path_msg_->header.stamp = step_time;
  path_msg_->header.frame_id = frame_id_;
  get_trajectory();
  m_rpp_command_publisher_->publish(*tj_msg_);
  m_trajectory_publisher_->publish(*path_msg_);
  get_tree(step_time);
  m_path_viz_publisher_->publish(*marker_array_msg_);

  telem_msg_->stamp = step_time;
  get_telemetry();
  m_rpp_telemetry_publisher_->publish(*telem_msg_);
  auto stop = now();
  auto diff = stop - step_time;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), debug_, "Path planner took %f seconds", diff.seconds());
}

void RacePathPlannerNode::get_telemetry()
{
  m_frenet_path_planner_->get_telemetry(telem_msg_);
}

void RacePathPlannerNode::get_trajectory()
{
  tj_msg_->trajectory.poses.clear();
  path_msg_->poses.clear();
  m_frenet_path_planner_->find_best_path();
  m_frenet_path_planner_->get_best_path(tj_msg_, path_msg_);
}

void RacePathPlannerNode::get_tree(const rclcpp::Time & step_time)
{
  marker_array_msg_->markers.clear();
  m_frenet_path_planner_->get_tree(marker_array_msg_, step_time, frame_id_);
}

}    // namespace race_path_planner
}  // namespace race

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<race::race_path_planner::RacePathPlannerNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
