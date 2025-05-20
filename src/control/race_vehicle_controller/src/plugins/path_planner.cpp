// Copyright 2022 AI Racing Tech
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// Creates paths for the RVC to track


#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <limits>
#include <algorithm>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "transform_helper/transform_helper.hpp"
#include "std_msgs/msg/float64.hpp"

#include "CGAL/Line_2.h"
#include "CGAL/Point_2.h"
#include "CGAL/Cartesian.h"
#include "CGAL/squared_distance_2.h"

#include "ttl.hpp"
#include "ttl_tree.hpp"

#include "race_vehicle_controller/plugin.hpp"

using race::ttl::load_all;
using race::ttl::dec_waypoint_index;
using race::ttl::inc_waypoint_index;
using race::ttl::Ttl;
using race::ttl::TtlArray;
using race::ttl::TtlTree;
using race::ttl::TtlIndex;
using race::ttl::Position;
using race::ttl::Waypoint;

namespace race
{
struct PlannerConfig
{
  std::string ttl_file_path;
  double step_interval_sec;
  double lookahead_speed_ratio;
  double lateral_error_ratio;
  std::vector<uint8_t> lateral_error_ratio_bypass_ttl;
  double lookahead_max_distance_m;
  double lookahead_min_distance_m;
  double distance_between_waypoints_m;
  double no_acc_above_lateral_error_m;
  double min_maintained_speed_mps;
};

struct PlannerState
{
  Position projection {};
  Position lookahead {};
  double cross_track_error = 0.0;
  double lookahead_distance = 0.0;
  size_t lookahead_index = 0;
  double target_speed = 0.0;
  size_t index;
  TtlIndex current_ttl_index;
  double current_heading;
  Position current_location;
  double speed_limit;
  double scale_factor;
  double current_speed;
  double wayline_delta;
};

class PathPlannerPlugin : public RvcPlugin
{
public:
  bool configure() override
  {
    pub_path_ = node().create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS{10});
    pub_ttl_ = node().create_publisher<nav_msgs::msg::Path>("ttl", rclcpp::QoS{10});
    pub_curr_delay_ =
      node().create_publisher<std_msgs::msg::Float64>("curr_delay", rclcpp::QoS{10});
    const auto valid_ttls = node().declare_parameter<std::vector<int64_t>>(
      "planner.lookahead.lateral_error_ratio_bypass_ttl");
    delay = node().declare_parameter<double>("planner.delay", 0.0);
    enable_delay_shifting_ = node().declare_parameter<bool>("planner.enable_delay_shifting", false);
    bank_angle_thres = node().declare_parameter<double>("planner.bank_angle_thres", 0.02);
    smoothness_factor = node().declare_parameter<double>("planner.smoothness_factor", 0.02);
    curr_delay = 0.;
    config_ = PlannerConfig{
      const_config().ttl_directory,
      node().declare_parameter<double>("planner.step_interval_sec", 0.01),
      node().declare_parameter<double>("planner.lookahead.speed_ratio", 2.0),
      node().declare_parameter<double>("planner.lookahead.lateral_error_ratio", 5.0),
      std::vector<uint8_t>(valid_ttls.begin(), valid_ttls.end()),
      node().declare_parameter<double>("planner.lookahead.max_distance_m", 100.0),
      node().declare_parameter<double>("planner.lookahead.min_distance_m", 5.0),
      node().declare_parameter<double>("planner.distance_between_waypoints_m", 0.1),
      node().declare_parameter<double>("planner.no_acc_above_lateral_error_m", 2.0),
      node().declare_parameter<double>("planner.min_maintained_speed_mps", 5.0)
    };
    tree_ = std::make_unique<TtlTree>(config_.ttl_file_path.c_str());
    state_.current_ttl_index = static_cast<TtlIndex>(*tree_->get_valid_ttl_index_set().begin());
    state_ = PlannerState();


    ttl_vis_timer_ = rclcpp::create_timer(
      &node(), node().get_clock(), std::chrono::duration<float>(1.0), [this] {
        if (current_ttl_vis_) {pub_ttl_->publish(*current_ttl_vis_);}
      });
    return true;
  }

  void on_kinematic_update() override
  {
    if (!const_state().ttl_cmd) {
      return;
    }
    const auto & msg = *const_state().kin_state;
    const auto location = Position {msg.pose.pose.position.x, msg.pose.pose.position.y};
    const auto heading = TransformHelper::heading_from_quaternion(msg.pose.pose.orientation);
    set_location(location, heading);
    set_current_speed(msg.speed_mps);
  }

  void on_ttl_command_update() override
  {
    const auto & msg = *const_state().ttl_cmd;
    const auto prev_ttl_index = state_.current_ttl_index;
    set_current_ttl_index(static_cast<TtlIndex>(msg.current_ttl_index));
    if (prev_ttl_index != state_.current_ttl_index || !current_ttl_vis_) {
      current_ttl_vis_ = get_global_ttl();
    }
    set_wayline_delta(msg.wayline_delta);
    auto target_speed = msg.target_speed;
    if (msg.stop_type.stop_type == race_msgs::msg::StopType::STOP_TYPE_EMERGENCY ||
      msg.stop_type.stop_type == race_msgs::msg::StopType::STOP_TYPE_IMMEDIATE)
    {
      target_speed = 0.0;
    }
    set_speed_limit(target_speed);
    state_.scale_factor = msg.target_waypoint_scale;
    state().telemetry->speed_limit = target_speed;
  }

  void on_trajectory_update() override
  {
    if (!const_state().all_input_received()) {
      return;
    }
    PathSharedPtr path = plan();
    state().telemetry->projection_x = get_state().projection.x;
    state().telemetry->projection_y = get_state().projection.y;
    state().telemetry->lookahead_x = get_state().lookahead.x;
    state().telemetry->lookahead_y = get_state().lookahead.y;
    state().telemetry->lookahead_distance = get_state().lookahead_distance;
    state().telemetry->lateral_error = get_state().cross_track_error;
    state().telemetry->target_speed = get_state().target_speed;
    state().telemetry->lookahead_index = get_state().lookahead_index;
    auto local_path = transform_path_to_local(
      path, const_state().kin_state->speed_mps,
      const_state().kin_state->car_yaw_rate);
    pub_path_->publish(*to_nav_path(path, "map"));
    state().path = local_path;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    const auto & name = param.get_name();
    const auto & type = param.get_type();
    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "planner.lookahead.speed_ratio") {
        get_config().lookahead_speed_ratio = param.as_double();
      } else if (name == "planner.lookahead.max_distance_m") {
        get_config().lookahead_max_distance_m = param.as_double();
      } else if (name == "planner.lookahead.min_distance_m") {
        get_config().lookahead_min_distance_m = param.as_double();
      } else if (name == "planner.no_acc_above_lateral_error_m") {
        get_config().no_acc_above_lateral_error_m = param.as_double();
      } else if (name == "planner.min_maintained_speed_mps") {
        get_config().min_maintained_speed_mps = param.as_double();
      } else if (name == "planner.lookahead.lateral_error_ratio") {
        get_config().lateral_error_ratio = param.as_double();
      } else if (name == "planner.delay") {
        delay = param.as_double();
      } else if (name == "planner.bank_angle_thres") {
        bank_angle_thres = param.as_double();
      } else if (name == "planner.smoothness_factor") {
        smoothness_factor = param.as_double();
      } else {
        return false;
      }
      return true;
    } else if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == "planner.enable_delay_shifting") {
        enable_delay_shifting_ = param.as_bool();
      } else {
        return false;
      }
      return true;
    }
    return false;
  }

private:
  void update_current_ttl_position()
  {
    if (tree_) {
      state_.index = tree_->find_closest_waypoint_index(
        get_current_ttl_index(), state_.current_location);
    }
  }

  void set_location(Position const & new_location, double heading)
  {
    state_.current_location = new_location;
    state_.current_heading = heading;
    update_current_ttl_position();
  }

  void set_current_ttl_index(const TtlIndex & new_ttl_index)
  {
    if (state_.current_ttl_index != new_ttl_index) {
      if (tree_->is_ttl_index_valid(static_cast<uint8_t>(new_ttl_index))) {
        state_.current_ttl_index = new_ttl_index;
        update_current_ttl_position();
      } else {
        RCLCPP_ERROR_THROTTLE(
          node().get_logger(),
          *node().get_clock(), 500,
          ("Boo. Invalid TTL index. Staying on TTL " + std::to_string(
            static_cast<int>(state_.current_ttl_index))).c_str());
      }
    }
  }

  void set_wayline_delta(const double & wayline_delta)
  {
    state_.wayline_delta = wayline_delta;
  }

  void set_speed_limit(const double & speed_limit)
  {
    state_.speed_limit = speed_limit;
  }

  void set_current_speed(const double & speed)
  {
    state_.current_speed = speed;
  }

  Position transform_position(
    Position const & position, double current_speed,
    double car_yaw_rate) const
  {
    auto pos = geometry_msgs::msg::PoseStamped();
    pos.header.frame_id = "map";
    pos.pose.position.x = position.x;
    pos.pose.position.y = position.y;

    // Construct map -> base_link transform
    auto transform = tf2::Transform();
    auto shifted_position = state_.current_location;
    if (enable_delay_shifting_) {
      shifted_position = Position(
        state_.current_location.x + ((current_speed * cos(state_.current_heading)) * curr_delay),
        state_.current_location.y + ((current_speed * sin(state_.current_heading)) * curr_delay)
      );
    }
    auto translation = tf2::Vector3{shifted_position.x, shifted_position.y, 0.0};
    transform.setOrigin(translation);
    double shifted_yaw = state_.current_heading;
    if (enable_delay_shifting_) {
      shifted_yaw = state_.current_heading + (car_yaw_rate) * curr_delay;
      while (shifted_yaw > M_PI) {
        shifted_yaw -= 2 * M_PI;
      }

      while (shifted_yaw <= -M_PI) {
        shifted_yaw += 2 * M_PI;
      }
    }
    auto rot = TransformHelper::quaternion_from_heading(shifted_yaw);
    transform.setRotation(rot);
    // Flip the transform to get base_link -> map
    transform = transform.inverse();
    // Apply the new transform
    auto geo_transform = geometry_msgs::msg::TransformStamped();
    geo_transform.child_frame_id = "map";
    geo_transform.header.frame_id = "base_link";
    geo_transform.transform = tf2::toMsg(transform);

    auto pos_in_ego = geometry_msgs::msg::PoseStamped();
    tf2::doTransform(pos, pos_in_ego, geo_transform);

    return Position {pos_in_ego.pose.position.x, pos_in_ego.pose.position.y};
  }

  double get_track_length() const
  {
    return get_current_ttl().header.total_distance;
  }

  TtlIndex get_current_ttl_index() const
  {
    return state_.current_ttl_index;
  }

  const Ttl & get_current_ttl() const
  {
    return tree_->get_ttl(state_.current_ttl_index);
  }

  PlannerConfig & get_config()
  {
    return config_;
  }

  PathSharedPtr plan()
  {
    PathSharedPtr path;
    PathSharedPtr path_shifted;
    if (!tree_) {
      return std::make_shared<Path>();
    }

    try {
      // Calculate lateral error
      state_.cross_track_error = race::ttl::get_cross_track_error(
        tree_->get_ttl(get_current_ttl_index()),
        state_.current_location,
        tree_->find_closest_waypoint_index(get_current_ttl_index(), state_.current_location));

      // Calculate lookahead distance
      auto lookahead_distance = std::clamp(
        state_.current_speed * config_.lookahead_speed_ratio, config_.lookahead_min_distance_m,
        config_.lookahead_max_distance_m);

      if (std::find(
          config_.lateral_error_ratio_bypass_ttl.begin(),
          config_.lateral_error_ratio_bypass_ttl.end(),
          static_cast<uint8_t>(get_current_ttl().header.number)) ==
        config_.lateral_error_ratio_bypass_ttl.end())
      {
        lookahead_distance = std::max(
          lookahead_distance, abs(
            state_.cross_track_error) * config_.lateral_error_ratio);
        lookahead_distance = std::clamp(
          lookahead_distance, config_.lookahead_min_distance_m,
          config_.lookahead_max_distance_m);
      }
      state_.lookahead_distance = lookahead_distance;

      // Calculate path and shifted path
      auto shifted_position = state_.current_location;
      if (enable_delay_shifting_) {
        shifted_position = Position(
          state_.current_location.x + ((const_state().kin_state->speed_mps * cos(
            state_.current_heading)) * curr_delay),
          state_.current_location.y + ((const_state().kin_state->speed_mps * sin(
            state_.current_heading)) * curr_delay)
        );
      }
      path = tree_->get_interpolated_trajectory(
        get_current_ttl_index(), state_.current_location, lookahead_distance);
      path_shifted = tree_->get_interpolated_trajectory(
        get_current_ttl_index(), shifted_position, lookahead_distance);

      // Update state
      state_.lookahead = path_shifted->back().location;
      state_.projection = path->front().location;

      // Publish current delay and update delay
      std_msgs::msg::Float64 curr_delay_msg;
      curr_delay_msg.data = curr_delay;
      pub_curr_delay_->publish(curr_delay_msg);
      double bank_angle_diff = path->back().bank_angle - path->front().bank_angle;
      if (abs(bank_angle_diff) > bank_angle_thres) {
        curr_delay = curr_delay + (-curr_delay) * smoothness_factor;
      } else {
        curr_delay = curr_delay + (delay - curr_delay) * smoothness_factor;
      }

      // Adjust speed limits
      double current_cycle_speed_limit = state_.speed_limit;
      double current_cycle_scale_factor = state_.scale_factor;

      if (abs(state_.cross_track_error) > config_.no_acc_above_lateral_error_m &&
        state_.speed_limit > 0.0)
      {
        current_cycle_speed_limit =
          std::max(state_.current_speed, config_.min_maintained_speed_mps);
        RCLCPP_WARN_THROTTLE(
          node().get_logger(),
          *node().get_clock(), 500, "Lateral error of %f exceeds threshold. Holding speed.",
          state_.cross_track_error);
      }
      for (auto & waypoint : *path) {
        if (waypoint.target_speed > 0.0) {
          waypoint.target_speed = std::min(
            current_cycle_speed_limit,
            waypoint.target_speed * current_cycle_scale_factor);
        } else {
          waypoint.target_speed = current_cycle_speed_limit;
        }
      }
      for (auto & waypoint : *path_shifted) {
        if (waypoint.target_speed > 0.0) {
          waypoint.target_speed = std::min(
            current_cycle_speed_limit,
            waypoint.target_speed * current_cycle_scale_factor);
        } else {
          waypoint.target_speed = current_cycle_speed_limit;
        }
      }
      state_.target_speed = path->back().target_speed;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        node().get_logger(),
        *node().get_clock(), 500, "Exception in path planning: %s", ex.what());
    }
    return path_shifted;
  }

  PathSharedPtr transform_path_to_local(
    const PathSharedPtr & path, double current_speed,
    double car_yaw_rate) const
  {
    auto path_tf = std::make_shared<Path>();
    for (const auto & wp : *path) {
      auto wp_tf = Waypoint(wp);
      wp_tf.location = transform_position(wp.location, current_speed, car_yaw_rate);
      wp_tf.target_yaw =
        TransformHelper::calc_yaw_difference<double>(state_.current_heading, wp.target_yaw);
      path_tf->push_back(wp_tf);
    }
    return path_tf;
  }

  PathSharedPtr get_local_ttl() const
  {
    auto path_tf = std::make_shared<Path>();
    const auto & wps = get_current_ttl().waypoints;
    for (uint32_t i = 0; i < get_current_ttl().header.loop; i++) {
      auto wp_tf = Waypoint();
      wp_tf.location = transform_position(wps[i].location, 0.0f, 0.0f);
      wp_tf.target_yaw = wps[i].target_yaw - state_.current_heading;
      wp_tf.target_radius = wps[i].target_radius;
      wp_tf.target_speed = wps[i].target_speed;
      path_tf->push_back(wp_tf);
    }
    return path_tf;
  }

  nav_msgs::msg::Path::SharedPtr get_global_ttl() const
  {
    auto path_msg = std::make_shared<nav_msgs::msg::Path>();
    path_msg->header.frame_id = "map";
    const auto & wps = get_current_ttl().waypoints;
    path_msg->poses.reserve(get_current_ttl().header.loop);
    for (uint32_t i = 0; i < get_current_ttl().header.loop; i++) {
      auto & pose = path_msg->poses.emplace_back();
      pose.header = path_msg->header;
      pose.pose.position.x = wps[i].location.x;
      pose.pose.position.y = wps[i].location.y;
      auto quat_tf2 = TransformHelper::quaternion_from_heading(wps[i].target_yaw);
      pose.pose.orientation = tf2::toMsg(quat_tf2);
    }
    return path_msg;
  }

  PlannerState & get_state()
  {
    return state_;
  }

  nav_msgs::msg::Path::SharedPtr to_nav_path(const PathSharedPtr path, const char * frame_id)
  {
    auto path_msg = std::make_shared<nav_msgs::msg::Path>();
    path_msg->header.stamp = node().now();
    path_msg->header.frame_id = frame_id;
    for (const auto & wp : *path) {
      auto pose = geometry_msgs::msg::PoseStamped();
      pose.header = path_msg->header;
      pose.pose.position.x = wp.location.x;
      pose.pose.position.y = wp.location.y;
      auto quat_tf2 = TransformHelper::quaternion_from_heading(wp.target_yaw);
      pose.pose.orientation = tf2::toMsg(quat_tf2);
      path_msg->poses.push_back(pose);
    }
    return path_msg;
  }

  TtlTree::UniquePtr tree_;
  PlannerConfig config_;
  PlannerState state_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ttl_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_curr_delay_;

  bool enable_delay_shifting_ = false;
  double delay;
  double curr_delay;
  double bank_angle_thres;
  double smoothness_factor;
  rclcpp::TimerBase::SharedPtr ttl_vis_timer_;
  nav_msgs::msg::Path::SharedPtr current_ttl_vis_;
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::PathPlannerPlugin, race::RvcPlugin)
