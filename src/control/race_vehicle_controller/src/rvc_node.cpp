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


#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "base_common/pubsub.hpp"

#include "pluginlib/class_loader.hpp"

#include "vehicle_model/ros_param_loader.hpp"
#include "race_vehicle_controller/rvc_node.hpp"
#include "race_msgs/msg/rvc_telemetry.hpp"
#include "transform_helper/transform_helper.hpp"

using std::chrono::duration;

namespace race
{

static constexpr auto MAX_DT_ERROR = 2;
enum PathPlanningMode
{
  INTERNAL = 0,
  EXTERNAL = 1
};

RvcNode::RvcNode(const rclcpp::NodeOptions & options)
: Node("rvc", options),
  m_plugin_loader_("race_vehicle_controller", "race::RvcPlugin")
{
  // Initialize members
  m_config_ = std::make_shared<RvcConfig>(
    RvcConfig{
      declare_parameter<double>("rvc.step_interval_sec", 0.01),
      declare_parameter<double>("rvc.max_front_wheel_angle_rad", 0.5236),
      declare_parameter<double>("rvc.wheelbase_m", 1.05),
      declare_parameter<double>("rvc.track_m", 0.9),
      declare_parameter<double>("rvc.vehicle_weight_kg", 200.0),
      declare_parameter<double>("rvc.turn_left_bias_deg", 0.0) * M_PI / 180.0,
      declare_parameter<std::string>("ttl_directory")
    });

  m_profiler_.set_window(
    std::max<size_t>(
      static_cast<size_t>(1.0 /
      m_config_->control_output_interval_sec), 20));
  m_state_ = std::make_shared<RvcState>();
  m_state_->telemetry = std::make_shared<race_msgs::msg::RvcTelemetry>();
  m_state_->diagnostics = std::make_shared<DiagnosticArray>();
  m_model_ = std::make_shared<race::vehicle_model::VehicleModel>(
    race::vehicle_model::VehicleModelConfig::SharedPtr());
  race::vehicle_model::load_parameters(this, *m_model_);

  // Load plugins
  const auto plugin_names = declare_parameter("plugins", std::vector<std::string>{});
  for (const auto & plugin_name : plugin_names) {
    RvcPlugin::SharedPtr new_plugin = m_plugin_loader_.createSharedInstance(plugin_name);
    m_plugins_.push_back(new_plugin);
  }
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->initialize(m_state_, m_config_, m_model_, this);
    });
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->configure();
    });

  // Publishers (to Interface)
  pubsub::publish_to(this, pub_raw_control_cmd_, "raw_command", rclcpp::QoS{1});

  // Publishers (to Telemetry)
  pubsub::publish_to(this, pub_telemetry_, "rvc_telemetry");

  // Publishers (to Diagnostics)
  // Best effort QoS for diagnostics
  pubsub::publish_to(this, pub_diag_, "diagnostics", rclcpp::QoS{10});

  // Path planning
  PathPlanningMode path_planning_mode =
    static_cast<PathPlanningMode>(declare_parameter<int64_t>("rvc.path_planning_mode", 0));
  if (path_planning_mode == PathPlanningMode::INTERNAL) {
    planner_timer_ = rclcpp::create_timer(
      this, get_clock(), duration<float>(
        declare_parameter<double>(
          "rvc.planning_interval_sec")), [this] {
        planner_timer_callback();
      });
  } else if (path_planning_mode == PathPlanningMode::EXTERNAL) {
    pubsub::subscribe_from(
      this,
      sub_path_, "manual_command",
      &RvcNode::on_path);
  }

  // Subscribers (from RDE)
  pubsub::subscribe_from(
    this,
    sub_trajectory_update_, "trajectory_update",
    &RvcNode::on_trajectory_update_received);
  pubsub::subscribe_from(
    this,
    sub_manual_control_, "manual_command",
    &RvcNode::on_vehicle_manual_command_received);

  // Subscribers (from Push2Pass)
  pubsub::subscribe_from(
    this,
    sub_push2pass_report_, "push2pass_report",
    &RvcNode::on_push2pass_received);

  // Subscribers (from Sensors)
  pubsub::subscribe_from(
    this,
    sub_kin_state_, "vehicle_kinematic_state",
    &RvcNode::on_kinematic_state_received);

  pubsub::subscribe_from(
    this,
    sub_wheel_speed_report_, "wheel_speed_report",
    &RvcNode::on_wheel_speed_received);

  pubsub::subscribe_from(
    this,
    sub_engine_report_, "engine_report",
    &RvcNode::on_engine_report_received);

  // Timers
  step_timer_ =
    rclcpp::create_timer(
    this, get_clock(), duration<float>(
      m_config_->control_output_interval_sec), [this] {step();});

  // Parameter callback
  callback_handle_ = add_on_set_parameters_callback(
    std::bind(&RvcNode::on_set_parameters, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult RvcNode::on_set_parameters(
  std::vector<rclcpp::Parameter> const & parameters)
{
  (void) parameters;
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;

  for (const auto & param : parameters) {
    // update rvc and path planner params
    const auto & name = param.get_name();
    const auto & type = param.get_type();
    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "rvc.turn_left_bias_deg") {
        m_config_->turn_left_bias_rad = param.as_double() * M_PI / 180.0;
      } else {
        goto plugin_params;
      }
    } else {
      goto plugin_params;
    }
    result.successful = true;
    continue;
    // update plugin params
plugin_params: result.successful = std::any_of(
      m_plugins_.begin(), m_plugins_.end(), [&param](RvcPlugin::SharedPtr & p) {
        return p->update_param(param);
      });
    if (!result.successful) {
      break;
    }
  }
  result.reason = result.successful ? "success" : "failure";
  return result;
}

void RvcNode::on_kinematic_state_received(const VehicleKinematicState::SharedPtr msg)
{
  m_state_->kin_state = msg;
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->on_kinematic_update();
    });
}

void RvcNode::on_wheel_speed_received(const WheelSpeedReport::SharedPtr msg)
{
  m_state_->wheel_speed_report = msg;
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->on_wheel_speed_update();
    });
}

void RvcNode::on_engine_report_received(const EngineReport::SharedPtr msg)
{
  m_state_->engine_report = msg;
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->on_engine_report_update();
    });
}

void RvcNode::on_trajectory_update_received(
  const TargetTrajectoryCommand::SharedPtr msg)
{
  m_state_->ttl_cmd = msg;
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->on_ttl_command_update();
    });
}

void RvcNode::on_push2pass_received(const Push2PassReport::SharedPtr msg)
{
  m_state_->push2pass_report = msg;
}

void RvcNode::on_vehicle_manual_command_received(const VehicleManualControlCommand::SharedPtr msg)
{
  m_state_->input_manual_cmd = msg;
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->on_manual_command_update();
    });
}

void RvcNode::on_path(const RacePathCommand::SharedPtr msg)
{
  const auto & nav_path = msg->trajectory;
  auto traj = std::make_shared<race::ttl::Path>();
  traj->reserve(nav_path.poses.size());
  for (const auto & pose : nav_path.poses) {
    auto & traj_pt = traj->emplace_back();
    traj_pt.location = Position{pose.pose.position.x, pose.pose.position.y};
    traj_pt.target_yaw = TransformHelper::heading_from_quaternion(pose.pose.orientation);
    traj_pt.target_speed = pose.pose.position.z;
  }
  m_state_->path = traj;
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->on_trajectory_update();
    });
}

void RvcNode::step()
{
  static size_t cycle_count = 1;
  const auto step_time = now();
  m_profiler_.start();
  try {
    if (last_step_time_.get_clock_type() == RCL_CLOCK_UNINITIALIZED) {
      last_step_time_ = step_time;
      return;
    }

    auto actual_dt = (step_time - last_step_time_).seconds();
    last_step_time_ = step_time;

    if (actual_dt >= MAX_DT_ERROR * m_config_->control_output_interval_sec) {
      RCLCPP_WARN(get_logger(), "Controller freq compromised. Actual DT (sec): %f", actual_dt);
    }
    m_state_->output_cmd = std::make_shared<VehicleControlCommand>();
    auto & cmd = *(m_state_->output_cmd);
    cmd.stamp = now();
    bool output_good = std::all_of(
      m_plugins_.begin(), m_plugins_.end(), [&cmd, this](RvcPlugin::SharedPtr & p) {
        try {
          return p->compute_control_command(cmd);
        } catch (const std::exception & ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(), 500, "Exception in %s compute_control_command(): %s",
            p->get_plugin_name(), ex.what());
          return false;
        }
      });
    output_good = output_good && std::all_of(
      m_plugins_.begin(), m_plugins_.end(), [&cmd, this](RvcPlugin::SharedPtr & p) {
        try {
          return p->modify_control_command(cmd);
        } catch (const std::exception & ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(), 500, "Exception in %s modify_control_command(): %s", p->get_plugin_name(),
            ex.what());
          return false;
        }
      });
    if (output_good) {
      std::for_each(
        m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
          p->on_command_output();
        });
      pub_raw_control_cmd_->publish(cmd);
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 500, "Exception in RvcNode::step(): %s",
      ex.what());
  }
  m_profiler_.end();
  if (cycle_count % m_profiler_.capacity() == 0) {
    using diagnostic_msgs::msg::KeyValue;
    const auto profile = m_profiler_.profile();
    auto & status = m_state_->diagnostics->status.emplace_back();
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "RVC";
    status.message = "Loop Profile";
    auto mean = KeyValue();
    mean.key = "mean (ms)";
    mean.value = std::to_string(profile.mean.count());
    auto max = KeyValue();
    max.key = "max (ms)";
    max.value = std::to_string(profile.max.count());
    auto min = KeyValue();
    min.key = "min (ms)";
    min.value = std::to_string(profile.min.count());
    status.values = {mean, max, min};
  }

  {
    // Send telemetry.
    m_state_->telemetry->stamp = step_time;
    m_state_->telemetry->cycle_sec =
      static_cast<float>((rclcpp::Time(m_state_->telemetry->stamp) - last_step_time_).seconds());
    pub_telemetry_->publish(*(m_state_->telemetry));
  }

  {
    // Send diagnostics.
    auto & msg = *m_state_->diagnostics;
    if (msg.status.size() > 0) {
      msg.header.stamp = step_time;
      pub_diag_->publish(*(m_state_->diagnostics));
      msg.status.clear();
    }
  }
  cycle_count += 1;
}

void RvcNode::planner_timer_callback()
{
  if (!m_state_->all_input_received()) {
    return;
  }
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->on_trajectory_update();
    });
}
}  // namespace race

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<race::RvcNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
