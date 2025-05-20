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
// limitations under the License.

/// \copyright Copyright 2022 Siddharth Saha
/// \file
/// \brief This file defines the frenet path planner class.

#ifndef RACE_PATH_PLANNER__FRENET_PATH_PLANNER_HPP_
#define RACE_PATH_PLANNER__FRENET_PATH_PLANNER_HPP_

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include <mutex>

#include "race_path_planner/polynomials.hpp"

#include "ttl.hpp"
#include "ttl_tree.hpp"

#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/stop_type.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_msgs/msg/rpp_telemetry.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "race_msgs/msg/race_path_command.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace race
{
namespace race_path_planner
{
struct FrenetPathParams
{
  std::string ttl_dir;
  double ds;
  double max_left_deviation;
  double max_right_deviation;
  double road_inc;
  double max_speed_low_deviation;
  double max_speed_high_deviation;
  double speed_inc;
  double max_lat_vel_low_deviation;
  double max_lat_vel_high_deviation;
  double lat_vel_inc;
  double t_period;
  double dt;
  double obj_safety_thresh;
  double boundary_safety_thresh;
  double detection_radius;
  double follow_desired_gap;
  double kp_gap;
  double kc_prev;
  double kj;
  double kd;
  double klat;
  double klon;
  double max_acc;
  double max_curv;
  double marker_size;
  double marker_lifetime;
  double max_yaw_diff;
  double min_speed;
  int n_threads;
  uint8_t center_ttl;
};

struct Obstacle
{
  double x;
  double y;
  double r;
};

struct RivalCar
{
  bool exists;
  bool should_follow;
  double current_gap;
  double target_gap;
  double speed;
};

struct InitialConditions
{
  double current_s;
  double current_d;
  double current_s_speed;
  double current_d_speed;
  double current_s_acc;
  double current_d_acc;
};

struct EndConditions
{
  double desired_s;
  double desired_d;
  double desired_speed;
  double desired_s_speed;
  double desired_d_speed;
  double desired_s_acc;
  double desired_d_acc;
};

enum class PathStatus : uint8_t
{
  PATH_NON_EXISTENT,
  PATH_OPTIMAL,
  PATH_PASSED,
  PATH_CONSTRAINT_VIOLATED,
  PATH_COLLISION
};

struct FrenetPath
{
  std::vector<double> t;
  std::vector<double> d_offset;
  std::vector<double> d_speed;
  std::vector<double> d_acc;
  std::vector<double> d_jerk;
  std::vector<double> s_pos;
  std::vector<double> s_speed;
  std::vector<double> s_acc;
  std::vector<double> s_jerk;

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> speed;
  std::vector<double> curvature;
  std::vector<double> ds;

  double Js, Jp, cd, cv, cf, dss;

  PathStatus path_ok;
};

class FrenetPathPlanner
{
public:
  explicit FrenetPathPlanner(FrenetPathParams & params);
  void update_params(FrenetPathParams & params);
  void get_best_path(
    race_msgs::msg::RacePathCommand::SharedPtr & traj_cmd,
    nav_msgs::msg::Path::SharedPtr & path_cmd);
  void get_tree(
    visualization_msgs::msg::MarkerArray::SharedPtr & marker_array,
    const rclcpp::Time & step_time, const std::string & frame_id);
  void get_telemetry(race_msgs::msg::RppTelemetry::SharedPtr & telem_msg);
  void find_best_path();
  void update_pose(race_msgs::msg::VehicleKinematicState::SharedPtr & pose_msg);
  void update_objects(autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr & obj_msg);
  void update_ttc(race_msgs::msg::TargetTrajectoryCommand::SharedPtr & ttc_msg);
  bool validate_ttl_index(const int64_t & ttl_index);

private:
  void frenet_optimal_planning(
    const InitialConditions & init_conds,
    const EndConditions & end_conds);
  void calc_lats(
    int start_iter, int end_iter, bool lock, const InitialConditions & init_conds,
    const EndConditions & end_conds);
  void calc_lons(
    int start_iter, int end_iter, bool lock, const InitialConditions & init_conds,
    const EndConditions & end_conds, double min_lon_speed, double max_lon_speed);
  void calc_lat(
    const InitialConditions & init_conds, const EndConditions & end_conds,
    FrenetPath & fp);
  void calc_lon(
    const InitialConditions & init_conds, const EndConditions & end_conds,
    FrenetPath & fp);
  void calc_global_path(
    FrenetPath & fp);
  void check_path(FrenetPath & g_path, double target_speed);
  void load_viz(
    visualization_msgs::msg::Marker & marker, const std_msgs::msg::ColorRGBA & color,
    const FrenetPath & path);
  double matching_cost(const FrenetPath & fp);
  std::optional<race::ttl::Position> to_global_pose(
    race::ttl::Position & frenet_pos);
  size_t find_largest_idx_before_lap_dist(double lap_dist_thresh);

  FrenetPathParams params_;

  std::vector<FrenetPath> paths_;
  std::vector<FrenetPath> lat_paths_;
  std::vector<FrenetPath> lon_paths_;
  FrenetPath best_path_;

  InitialConditions init_conds_;
  EndConditions end_conds_;
  uint64_t n_paths_generated_;
  uint64_t n_paths_non_existent_;
  uint64_t n_paths_constraint_violated_;
  uint64_t n_paths_collision_;
  uint64_t n_paths_passed_;
  bool any_acc_violated_;
  bool any_speed_violated_;
  bool any_curv_violated_;

  std::mutex mu_;

  std::vector<Obstacle> objects_;
  race::ttl::Position pos_;
  race::ttl::Position speed_decomposed_;
  double speed_;
  double yaw_;
  std::uint8_t strategy_type_;
  std::uint8_t area_type_;
  std::unordered_set<uint8_t> valid_idx_;
  race_msgs::msg::StopType stop_type_;
  double target_speed_;
  race::ttl::TtlIndex current_ttl_idx_;

  race::ttl::TtlTree::UniquePtr ttl_tree_;

  std::unordered_map<race::ttl::TtlIndex, visualization_msgs::msg::MarkerArray> ttl_viz_map_;

  RivalCar rival_car_;
};
}  // namespace race_path_planner
}  // namespace race

#endif  // RACE_PATH_PLANNER__FRENET_PATH_PLANNER_HPP_
