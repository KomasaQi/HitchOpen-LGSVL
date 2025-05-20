// Copyright 2022 Siddharth Saha

#include "race_path_planner/frenet_path_planner.hpp"

#include <algorithm>
#include <mutex>
#include <thread>
#include <limits>
#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "race_msgs/msg/strategy_type.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "transform_helper/transform_helper.hpp"
#include "race_msgs/msg/rpp_pose.hpp"
#include "race_msgs/msg/rpp_trajectory.hpp"

namespace race
{
namespace race_path_planner
{

FrenetPathPlanner::FrenetPathPlanner(FrenetPathParams & params)
{
  params_ = params;
  rival_car_.exists = false;
  ttl_tree_ = std::make_unique<race::ttl::TtlTree>(params_.ttl_dir.c_str());
  valid_idx_ = ttl_tree_->get_valid_ttl_index_set();
  if (!ttl_tree_->is_ttl_index_valid(params.center_ttl)) {
    std::runtime_error("Center TTL is invalid");
  }
  best_path_ = FrenetPath();
  best_path_.path_ok = PathStatus::PATH_NON_EXISTENT;
}

bool FrenetPathPlanner::validate_ttl_index(const int64_t & ttl_index)
{
  return ttl_tree_->is_ttl_index_valid(ttl_index);
}

void FrenetPathPlanner::frenet_optimal_planning(
  const InitialConditions & init_conds,
  const EndConditions & end_conds)
{
  std::vector<std::thread> threads;

  int num_threads = params_.n_threads;

  int total_iter_lat =
    static_cast<int>((params_.max_left_deviation + params_.max_right_deviation) /
    params_.road_inc) + 1;
  double min_lon_speed = 0.0;
  double max_lon_speed = 0.0;
  if (speed_ < end_conds.desired_s_speed) {
    min_lon_speed = std::max(0.0, speed_ - params_.max_speed_low_deviation);
    max_lon_speed = std::max(0.0, end_conds.desired_s_speed + params_.max_speed_high_deviation);
  } else {
    min_lon_speed = std::max(0.0, end_conds.desired_s_speed - params_.max_speed_low_deviation);
    max_lon_speed = std::max(0.0, speed_ + params_.max_speed_high_deviation);
  }
  int per_thread_iter_lat = static_cast<int>(total_iter_lat / num_threads);
  int total_iter_lon =
    static_cast<int>((max_lon_speed - min_lon_speed) /
    params_.speed_inc) + 1;
  int per_thread_iter_lon = static_cast<int>(total_iter_lon / num_threads);

  paths_.clear();
  lat_paths_.clear();
  lon_paths_.clear();

  if (num_threads > 1) {
    for (int i = 0; i < num_threads; i++) {
      if (i != params_.n_threads - 1) {
        threads.push_back(
          std::thread(
            &FrenetPathPlanner::calc_lats, this, i * per_thread_iter_lat,
            (i + 1) * per_thread_iter_lat, true, init_conds, end_conds));
      } else {
        threads.push_back(
          std::thread(
            &FrenetPathPlanner::calc_lats, this, i * per_thread_iter_lat,
            total_iter_lat, true, init_conds, end_conds));
      }
    }

    for (std::thread & t : threads) {
      t.join();
    }

    threads.clear();

    for (int i = 0; i < num_threads; i++) {
      if (i != params_.n_threads - 1) {
        threads.push_back(
          std::thread(
            &FrenetPathPlanner::calc_lons, this, i * per_thread_iter_lon,
            (i + 1) * per_thread_iter_lon, true, init_conds, end_conds, min_lon_speed,
            max_lon_speed));
      } else {
        threads.push_back(
          std::thread(
            &FrenetPathPlanner::calc_lons, this, i * per_thread_iter_lon,
            total_iter_lon, true, init_conds, end_conds, min_lon_speed, max_lon_speed));
      }
    }

    for (std::thread & t : threads) {
      t.join();
    }

  } else {
    calc_lats(0, total_iter_lat, false, init_conds, end_conds);
    calc_lons(0, total_iter_lon, false, init_conds, end_conds, min_lon_speed, max_lon_speed);
  }
  for (auto & lat_path : lat_paths_) {
    for (auto & lon_path : lon_paths_) {
      n_paths_generated_ += 1;
      FrenetPath fp;
      fp = lat_path;
      fp.s_pos = lon_path.s_pos;
      fp.s_speed = lon_path.s_speed;
      fp.s_acc = lon_path.s_acc;
      fp.s_jerk = lon_path.s_jerk;
      fp.Js = lon_path.Js;
      fp.dss = lon_path.dss;
      fp.cv = lon_path.cv;
      fp.cf = (params_.klat * fp.cd) + (params_.klon * fp.cv);
      calc_global_path(fp);
      if (fp.path_ok == PathStatus::PATH_PASSED) {
        fp.cf += matching_cost(fp);
        check_path(fp, end_conds.desired_speed);
      } else {
        n_paths_non_existent_ += 1;
      }
      paths_.push_back(fp);
    }
  }
  double lowest_cost = FLT_MAX;
  int best_idx = -1;
  for (size_t i = 0; i < paths_.size(); i++) {
    if (paths_.at(i).path_ok == PathStatus::PATH_PASSED) {
      n_paths_passed_ += 1;
      if (paths_.at(i).cf < lowest_cost) {
        best_idx = i;
        lowest_cost = paths_.at(i).cf;
      }
    }
  }
  if (best_idx >= 0) {
    paths_.at(best_idx).path_ok = PathStatus::PATH_OPTIMAL;
    best_path_ = paths_.at(best_idx);
    best_path_.path_ok = PathStatus::PATH_OPTIMAL;
  } else {
    best_path_ = FrenetPath();
    best_path_.path_ok = PathStatus::PATH_NON_EXISTENT;
  }
}

void FrenetPathPlanner::calc_lats(
  int start_iter, int end_iter, bool lock,
  const InitialConditions & init_conds,
  const EndConditions & end_conds)
{
  for (double di = -params_.max_left_deviation + (start_iter * params_.road_inc);
    di < -params_.max_left_deviation + (end_iter * params_.road_inc) &&
    di < params_.max_right_deviation; di += params_.road_inc)
  {
    for (double vi = -params_.max_lat_vel_low_deviation; vi < params_.max_lat_vel_high_deviation;
      vi += params_.lat_vel_inc)
    {
      FrenetPath fp;
      EndConditions end_conds_modified = end_conds;
      end_conds_modified.desired_d = di;
      end_conds_modified.desired_d_speed = vi;
      calc_lat(init_conds, end_conds_modified, fp);
      if (lock) {
        mu_.lock();
        lat_paths_.push_back(fp);
        mu_.unlock();
      } else {
        lat_paths_.push_back(fp);
      }
    }
  }
}

void FrenetPathPlanner::calc_lons(
  int start_iter, int end_iter, bool lock,
  const InitialConditions & init_conds,
  const EndConditions & end_conds, double min_lon_speed, double max_lon_speed)
{
  for (double vi =
    min_lon_speed + (start_iter * params_.speed_inc);
    vi <
    min_lon_speed + (end_iter * params_.speed_inc) &&
    vi < max_lon_speed; vi += params_.speed_inc)
  {
    FrenetPath fp;
    EndConditions end_conds_modified = end_conds;
    end_conds_modified.desired_s_speed = vi;
    calc_lon(init_conds, end_conds_modified, fp);
    if (lock) {
      mu_.lock();
      lon_paths_.push_back(fp);
      mu_.unlock();
    } else {
      lon_paths_.push_back(fp);
    }
  }
}

void FrenetPathPlanner::calc_lat(
  const InitialConditions & init_conds,
  const EndConditions & end_conds, FrenetPath & fp)
{
  int n = 1 + (params_.t_period / params_.dt);
  QuinticPolynomialParams quintic_lat_params {init_conds.current_d, init_conds.current_d_speed,
    init_conds.current_d_acc, end_conds.desired_d, end_conds.desired_d_speed,
    end_conds.desired_d_acc, params_.t_period};
  QuinticPolynomial quintic_lat(quintic_lat_params);

  for (int te = 0; te < n; te++) {
    double t = te * params_.dt;
    fp.d_offset.push_back(quintic_lat.calc_point(t));
    fp.d_speed.push_back(quintic_lat.calc_first_derivative(t));
    fp.d_acc.push_back(quintic_lat.calc_second_derivative(t));
    fp.d_jerk.push_back(quintic_lat.calc_third_derivative(t));
  }

  std::vector<double> d_jerk_vec = fp.d_jerk;
  fp.Jp = std::inner_product(d_jerk_vec.begin(), d_jerk_vec.end(), d_jerk_vec.begin(), 0);
  fp.cd = (params_.kj * fp.Jp) + (params_.kd * std::pow(fp.d_offset.back(), 2));
}

void FrenetPathPlanner::calc_lon(
  const InitialConditions & init_conds,
  const EndConditions & end_conds, FrenetPath & fp)
{
  int n = 1 + (params_.t_period / params_.dt);
  QuinticPolynomialParams quintic_lon_params {init_conds.current_s, init_conds.current_s_speed,
    init_conds.current_s_acc, end_conds.desired_s, end_conds.desired_s_speed,
    end_conds.desired_s_acc, params_.t_period};
  QuarticPolynomial quartic_lon(quintic_lon_params);

  for (int te = 0; te < n; te++) {
    double t = te * params_.dt;
    fp.s_pos.push_back(quartic_lon.calc_point(t));
    fp.s_speed.push_back(quartic_lon.calc_first_derivative(t));
    fp.s_acc.push_back(quartic_lon.calc_second_derivative(t));
    fp.s_jerk.push_back(quartic_lon.calc_third_derivative(t));
  }

  std::vector<double> s_jerk_vec = fp.s_jerk;
  fp.Js = std::inner_product(s_jerk_vec.begin(), s_jerk_vec.end(), s_jerk_vec.begin(), 0);
  fp.dss = -1.0 * std::pow((fp.s_pos.front() - fp.s_pos.back()), 2);
  fp.cv = (params_.kj * fp.Js) + (params_.kd * fp.dss);
}


size_t FrenetPathPlanner::find_largest_idx_before_lap_dist(double lap_dist_thresh)
{
  int start = 0, end = ttl_tree_->get_ttl(current_ttl_idx_).header.loop - 1, predec = 0;
  int mid = 0;
  while (start <= end) {
    mid = start + (end - start) / 2;
    auto track_prog = ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(mid).dist_to_sf_bwd;
    if (track_prog < lap_dist_thresh) {
      start = mid + 1;
      predec = mid;
    } else if (track_prog > lap_dist_thresh) {
      end = mid - 1;
    } else {
      return mid;
    }
  }
  return predec;
}

std::optional<race::ttl::Position> FrenetPathPlanner::to_global_pose(
  race::ttl::Position & frenet_pos)
{
  race::ttl::Position global_pos;
  auto track_prog = std::fmod(
    frenet_pos.x, ttl_tree_->get_ttl(
      current_ttl_idx_).header.total_distance);
  auto closest_from_back_track_prog_idx = find_largest_idx_before_lap_dist(track_prog);
  auto target_idx = race::ttl::inc_waypoint_index(
    ttl_tree_->get_ttl(
      current_ttl_idx_), closest_from_back_track_prog_idx);
  double prev_track_prog = ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(
    closest_from_back_track_prog_idx).dist_to_sf_bwd;
  double target_track_prog = ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(
    target_idx).dist_to_sf_bwd;
  double interpolation_ratio = 0.0;
  if (target_track_prog > prev_track_prog) {
    interpolation_ratio = (track_prog - prev_track_prog) / (target_track_prog - prev_track_prog);
  }
  auto ref_pose = race::ttl::interpolate_positions(
    ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(
      closest_from_back_track_prog_idx).location, ttl_tree_->get_ttl(
      current_ttl_idx_).waypoints.at(target_idx).location, interpolation_ratio);
  global_pos.x = ref_pose.x +
    (ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(closest_from_back_track_prog_idx).
    normal_x * frenet_pos.y);
  global_pos.y = ref_pose.y +
    (ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(closest_from_back_track_prog_idx).
    normal_y * frenet_pos.y);
  return global_pos;
}

double FrenetPathPlanner::matching_cost(const FrenetPath & fp)
{
  if (best_path_.path_ok != PathStatus::PATH_OPTIMAL) {
    return 0.0;
  }
  size_t num_points_comparison = std::min(best_path_.x.size(), fp.x.size());
  if (num_points_comparison == 0) {
    return 0.0;
  }
  double deflating_cost = params_.kc_prev;
  double deflating_rate = params_.kc_prev / num_points_comparison;
  double total_cost = 0.0;
  for (size_t i = 0; i < num_points_comparison; i++) {
    double pair_dist = std::hypot(best_path_.x.at(i) - fp.x.at(i), best_path_.y.at(i) - fp.y.at(i));
    total_cost += (deflating_cost * pair_dist);
    deflating_cost -= deflating_rate;
  }
  return total_cost;
}

void FrenetPathPlanner::calc_global_path(FrenetPath & fp)
{
  fp.path_ok = PathStatus::PATH_PASSED;
  for (size_t i = 0; i < fp.s_pos.size(); i++) {
    race::ttl::Position frenet_pos;
    frenet_pos.x = fp.s_pos.at(i);
    frenet_pos.y = fp.d_offset.at(i);
    auto global_pos = to_global_pose(frenet_pos);
    if (!global_pos) {
      fp.path_ok = PathStatus::PATH_NON_EXISTENT;
      return;
    }
    fp.x.push_back(global_pos.value().x);
    fp.y.push_back(global_pos.value().y);
  }
  if (fp.x.size() <= 1 || fp.x.size() >= ttl_tree_->get_ttl(current_ttl_idx_).header.loop) {
    fp.path_ok = PathStatus::PATH_NON_EXISTENT;
    return;
  }
  for (size_t i = 0; i < fp.x.size() - 1; i++) {
    double dx = fp.x.at(i + 1) - fp.x.at(i);
    double dy = fp.y.at(i + 1) - fp.y.at(i);
    fp.yaw.push_back(std::atan2(dy, dx));
    fp.ds.push_back(std::hypot(dx, dy));
  }
  if (fp.x.size() >= 2 && fp.y.size() >= 2) {
    double dx = fp.x.front() - fp.x.back();
    double dy = fp.y.front() - fp.y.back();
    fp.yaw.push_back(std::atan2(dy, dx));
    fp.ds.push_back(std::hypot(dx, dy));
  }

  for (size_t i = 0; i < fp.yaw.size() - 1; i++) {
    if (fp.ds.at(i) != 0) {
      fp.curvature.push_back(
        TransformHelper::calc_yaw_difference(
          fp.yaw.at(i + 1), fp.yaw.at(
            i)) / fp.ds.at(i));
    }
  }
  if (fp.yaw.size() >= 2) {
    fp.curvature.push_back(
      TransformHelper::calc_yaw_difference(
        fp.yaw.front(), fp.yaw.back()) / fp.ds.back());
  }
}

void FrenetPathPlanner::check_path(FrenetPath & g_path, double target_speed)
{
  auto target_ttl = ttl_tree_->get_ttl(static_cast<race::ttl::TtlIndex>(current_ttl_idx_));
  auto center_ttl = ttl_tree_->get_ttl(static_cast<race::ttl::TtlIndex>(params_.center_ttl));
  g_path.path_ok = PathStatus::PATH_PASSED;
  if (g_path.path_ok == PathStatus::PATH_PASSED) {
    for (size_t i = 0; i < g_path.x.size(); i++) {
      race::ttl::Position path_pos{g_path.x.at(i), g_path.y.at(i)};
      auto wp_index =
        ttl_tree_->find_closest_waypoint_index(
        static_cast<race::ttl::TtlIndex>(current_ttl_idx_), path_pos);
      bool boundary_danger = (std::hypot(
          target_ttl.waypoints.at(wp_index).right_bound.x - g_path.x.at(i),
          target_ttl.waypoints.at(wp_index).right_bound.y - g_path.y.at(i)) <
        params_.boundary_safety_thresh) || (std::hypot(
          target_ttl.waypoints.at(wp_index).left_bound.x - g_path.x.at(i),
          target_ttl.waypoints.at(wp_index).left_bound.y - g_path.y.at(i)) <
        params_.boundary_safety_thresh);
      if (boundary_danger) {
        g_path.path_ok = PathStatus::PATH_COLLISION;
        n_paths_collision_ += 1;
        return;
      } else {
        for (auto ob : objects_) {
          if (std::hypot(ob.x - g_path.x.at(i), ob.y - g_path.y.at(i)) < ob.r) {
            g_path.path_ok = PathStatus::PATH_COLLISION;
            n_paths_collision_ += 1;
            return;
          }
        }
      }
    }
  }

  if (g_path.path_ok == PathStatus::PATH_PASSED) {
    if (area_type_ != race_msgs::msg::StrategyType::ENTIRE_TRACK) {
      race::ttl::Position path_pos{g_path.x.back(), g_path.y.back()};
      auto wp_index =
        ttl_tree_->find_closest_waypoint_index(
        static_cast<race::ttl::TtlIndex>(params_.
        center_ttl), path_pos);
      double cte = race::ttl::get_cross_track_error(center_ttl, path_pos, wp_index);
      if (area_type_ == race_msgs::msg::StrategyType::LEFT_HALF && cte <= 0) {
        g_path.path_ok = PathStatus::PATH_COLLISION;
        n_paths_collision_ += 1;
        return;
      } else if (area_type_ == race_msgs::msg::StrategyType::RIGHT_HALF && cte >= 0) {
        g_path.path_ok = PathStatus::PATH_COLLISION;
        n_paths_collision_ += 1;
        return;
      }
    }
  }
  bool speed_violated = std::any_of(
    g_path.s_speed.begin(), g_path.s_speed.end(),
    [this, &target_speed](int i) {
      return std::abs(i) > std::max(target_speed + params_.max_speed_high_deviation, speed_);
    });
  bool accel_violated = std::any_of(
    g_path.s_acc.begin(), g_path.s_acc.end(),
    [this](int i) {return std::abs(i) > params_.max_acc;});
  bool curvature_violated = std::any_of(
    g_path.curvature.begin(), g_path.curvature.end(),
    [this](int i) {return std::abs(i) > params_.max_curv;});
  if (speed_violated) {
    g_path.path_ok = PathStatus::PATH_CONSTRAINT_VIOLATED;
    any_speed_violated_ = true;
    n_paths_constraint_violated_ += 1;
  } else if (accel_violated) {
    g_path.path_ok = PathStatus::PATH_CONSTRAINT_VIOLATED;
    any_acc_violated_ = true;
    n_paths_constraint_violated_ += 1;
  } else if (curvature_violated) {
    g_path.path_ok = PathStatus::PATH_CONSTRAINT_VIOLATED;
    any_curv_violated_ = true;
    n_paths_constraint_violated_ += 1;
  } else {
    if (g_path.yaw.size() > 0) {
      if (std::abs(
          TransformHelper::calc_yaw_difference(
            yaw_,
            g_path.yaw.front())) > params_.max_yaw_diff)
      {
        g_path.path_ok = PathStatus::PATH_CONSTRAINT_VIOLATED;
        any_curv_violated_ = true;
        n_paths_constraint_violated_ += 1;
      }
    }
  }
}

void FrenetPathPlanner::get_best_path(
  race_msgs::msg::RacePathCommand::SharedPtr & traj_cmd,
  nav_msgs::msg::Path::SharedPtr & path_cmd)
{
  traj_cmd->target_speed = target_speed_;
  traj_cmd->stop_type = stop_type_;
  traj_cmd->current_ttl_index = static_cast<uint8_t>(current_ttl_idx_);
  if (best_path_.path_ok != PathStatus::PATH_OPTIMAL) {
    traj_cmd->path_exists = false;
    return;
  } else {
    traj_cmd->path_exists = true;
    race_msgs::msg::RppTrajectory rpp_traj;
    rpp_traj.header = path_cmd->header;
    for (size_t i = 0; i < best_path_.x.size(); i++) {
      geometry_msgs::msg::PoseStamped pose;
      race_msgs::msg::RppPose rpp_pose;
      rpp_pose.header = path_cmd->header;
      pose.header = path_cmd->header;
      pose.pose.position.x = best_path_.x.at(i);
      pose.pose.position.y = best_path_.y.at(i);
      if (i != best_path_.x.size() - 1) {
        pose.pose.orientation =
          tf2::toMsg(TransformHelper::quaternion_from_heading(best_path_.yaw.at(i)));
      } else {
        if (i > 0) {
          pose.pose.orientation =
            tf2::toMsg(TransformHelper::quaternion_from_heading(best_path_.yaw.at(i - 1)));
        }
      }
      rpp_pose.pose = pose.pose;
      rpp_pose.speed_mps = std::hypot(best_path_.s_speed.at(i), best_path_.d_speed.at(i));
      rpp_pose.acc_mps2 = best_path_.s_acc.at(i);
      rpp_pose.curvature_radm = best_path_.curvature.at(i);
      rpp_traj.poses.push_back(rpp_pose);
      path_cmd->poses.push_back(pose);
    }
    traj_cmd->trajectory = rpp_traj;
  }
}

void FrenetPathPlanner::get_tree(
  visualization_msgs::msg::MarkerArray::SharedPtr & marker_array,
  const rclcpp::Time & step_time, const std::string & frame_id)
{
  size_t i = 0;
  for (auto & path : paths_) {
    if (path.path_ok != PathStatus::PATH_NON_EXISTENT) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "/" + frame_id;
      marker.header.stamp = step_time;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(params_.marker_lifetime);
      marker.id = i;
      i++;
      marker.ns = "path_viz";
      marker.pose.orientation.w = 1.0;
      marker.scale.x = params_.marker_size;
      marker.scale.y = params_.marker_size;
      marker.scale.z = params_.marker_size;
      std_msgs::msg::ColorRGBA color;
      color.a = 1.0;
      switch (path.path_ok) {
        case PathStatus::PATH_OPTIMAL:
          color.r = 0.0f;
          color.g = 1.0f;
          color.b = 0.0f;
          marker.scale.x = params_.marker_size * 3.0;
          marker.scale.y = params_.marker_size * 3.0;
          marker.scale.z = params_.marker_size * 3.0;
          break;
        case PathStatus::PATH_PASSED:
          color.r = 1.0f;
          color.g = 1.0f;
          color.b = 0.0f;
          break;
        case PathStatus::PATH_CONSTRAINT_VIOLATED:
          color.r = 1.0f;
          color.g = 140.0 / 255.0;
          color.b = 0.0f;
          marker.scale.x = params_.marker_size * 0.5;
          marker.scale.y = params_.marker_size * 0.5;
          marker.scale.z = params_.marker_size * 0.5;
          break;
        case PathStatus::PATH_COLLISION:
          color.r = 1.0f;
          color.g = 0.0f;
          color.b = 0.0f;
          marker.scale.x = params_.marker_size * 0.5;
          marker.scale.y = params_.marker_size * 0.5;
          marker.scale.z = params_.marker_size * 0.5;
          break;
        default:
          break;
      }
      load_viz(marker, color, path);
      marker_array->markers.push_back(marker);
    }
  }
}

void FrenetPathPlanner::get_telemetry(race_msgs::msg::RppTelemetry::SharedPtr & telem_msg)
{
  telem_msg->x = pos_.x;
  telem_msg->y = pos_.y;
  telem_msg->speed = speed_;
  telem_msg->target_speed = target_speed_;
  telem_msg->stop_type = stop_type_;
  telem_msg->ttl_index = static_cast<uint8_t>(current_ttl_idx_);
  race_msgs::msg::StrategyType ros_strategy;
  ros_strategy.strategy_type = strategy_type_;
  telem_msg->strategy_type = ros_strategy;
  telem_msg->current_s = init_conds_.current_s;
  telem_msg->current_s_speed = init_conds_.current_s_speed;
  telem_msg->current_s_acc = init_conds_.current_s_acc;
  telem_msg->desired_s = end_conds_.desired_s;
  telem_msg->desired_s_speed = end_conds_.desired_s_speed;
  telem_msg->desired_s_acc = end_conds_.desired_s_acc;
  telem_msg->desired_speed = end_conds_.desired_speed;
  if (best_path_.path_ok != PathStatus::PATH_NON_EXISTENT) {
    telem_msg->calculated_x = (best_path_.x.front() - pos_.x) < 0.1;
    telem_msg->calculated_y = (best_path_.y.front() - pos_.y) < 0.1;
  }
  telem_msg->current_d = init_conds_.current_d;
  telem_msg->current_d_speed = init_conds_.current_d_speed;
  telem_msg->current_d_acc = init_conds_.current_d_acc;
  telem_msg->desired_d = end_conds_.desired_d;
  telem_msg->desired_d_speed = end_conds_.desired_d_speed;
  telem_msg->desired_d_acc = end_conds_.desired_d_acc;
  telem_msg->n_paths_collision = n_paths_collision_;
  telem_msg->n_paths_constraint_violated = n_paths_constraint_violated_;
  telem_msg->n_paths_generated = n_paths_generated_;
  telem_msg->n_paths_non_existent = n_paths_non_existent_;
  telem_msg->n_paths_passed = n_paths_passed_;
  telem_msg->any_acc_violated = any_acc_violated_;
  telem_msg->any_speed_violated = any_speed_violated_;
  telem_msg->any_curv_violated = any_curv_violated_;
  telem_msg->optimal_path_exists = (best_path_.path_ok == PathStatus::PATH_OPTIMAL);
}

void FrenetPathPlanner::load_viz(
  visualization_msgs::msg::Marker & marker,
  const std_msgs::msg::ColorRGBA & color, const FrenetPath & path)
{
  marker.color = color;
  for (size_t i = 0; i < path.x.size(); i++) {
    geometry_msgs::msg::Point point;
    point.x = path.x.at(i);
    point.y = path.y.at(i);
    marker.points.push_back(point);
  }
}

void FrenetPathPlanner::find_best_path()
{
  InitialConditions init_conds;
  n_paths_generated_ = 0;
  n_paths_non_existent_ = 0;
  n_paths_constraint_violated_ = 0;
  n_paths_collision_ = 0;
  n_paths_passed_ = 0;
  std::vector<size_t> closest_pts;
  size_t num_pts_query = 2;
  race::ttl::Waypoint prev_wp, target_wp, projected_wp;
  ttl_tree_->find_closest_waypoint_indices(current_ttl_idx_, pos_, num_pts_query, closest_pts);
  auto next_idx = race::ttl::inc_waypoint_index(
    ttl_tree_->get_ttl(
      current_ttl_idx_), closest_pts.front());
  if (next_idx == closest_pts.back()) {
    prev_wp = ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(closest_pts.front());
    target_wp =
      ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(closest_pts.back());
  } else {
    prev_wp = ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(closest_pts.back());
    target_wp =
      ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(closest_pts.front());
  }

  init_conds.current_d = -1.0 * race::ttl::get_cross_track_error(
    pos_, prev_wp.location, target_wp.location);
  ttl_tree_->get_projection(prev_wp, target_wp, projected_wp, pos_);
  double dist_to_front_pt = std::hypot(
    projected_wp.location.x - target_wp.location.x,
    projected_wp.location.y - target_wp.location.y);
  double dist_to_back_pt = std::hypot(
    projected_wp.location.x - prev_wp.location.x,
    projected_wp.location.y - prev_wp.location.y);
  double dist_bw_pts = std::hypot(
    target_wp.location.x - prev_wp.location.x,
    target_wp.location.y - prev_wp.location.y);
  double back_ratio = dist_to_back_pt / dist_bw_pts;
  double front_ratio = dist_to_front_pt / dist_bw_pts;
  init_conds.current_s = prev_wp.dist_to_sf_bwd + (back_ratio * dist_bw_pts);
  // double global_yaw = path_map_.at(current_ttl_idx_).ryaw.at(min_id);
  // double global_curvature = path_map_.at(current_ttl_idx_).rk.at(min_id);
  // double delta_theta = TransformHelper::calc_yaw_difference(yaw_, global_yaw);
  // std::max(0.0, speed_ * std::sin(delta_theta));
  init_conds.current_d_speed = 0.0;
  // / (1 - global_curvature * c_d);
  init_conds.current_s_speed = std::max(params_.min_speed, speed_);
  init_conds.current_d_acc = 0.0;
  init_conds.current_s_acc = 0.0;
  EndConditions end_conds;
  end_conds.desired_s_acc = 0.0;
  end_conds.desired_d_acc = 0.0;
  end_conds.desired_d_speed = 0.0;
  switch (strategy_type_) {
    case race_msgs::msg::StrategyType::CRUISE_CONTROL:
      end_conds.desired_speed = target_speed_;
      end_conds.desired_s_speed = target_speed_;
      end_conds.desired_d = 0.0;
      // end_conds.desired_s = init_conds.current_s + (target_speed_ * params_.t_period);
      break;
    case race_msgs::msg::StrategyType::FOLLOW_MODE:
      end_conds.desired_speed = std::max(
        0.0, rival_car_.speed - params_.kp_gap *
        (rival_car_.target_gap - rival_car_.current_gap));
      end_conds.desired_s_speed = std::max(
        0.0, rival_car_.speed - params_.kp_gap *
        (rival_car_.target_gap - rival_car_.current_gap));
      end_conds.desired_d = 0.0;
      break;
    default:
      end_conds.desired_speed = 0.0;
      end_conds.desired_s_speed = 0.0;
      end_conds.desired_d = 0.0;
      break;
  }
  init_conds_ = init_conds;
  end_conds_ = end_conds;
  frenet_optimal_planning(init_conds, end_conds);
}

void FrenetPathPlanner::update_params(FrenetPathParams & params)
{
  params_ = params;
}

void FrenetPathPlanner::update_objects(
  autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr & obj_msg)
{
  objects_.clear();
  for (auto obj : obj_msg->objects) {
    Obstacle ob =
    {obj.kinematics.pose_with_covariance.pose.position.x,
      obj.kinematics.pose_with_covariance.pose.position.y, params_.obj_safety_thresh};
    if (std::hypot(
        ob.x - pos_.x,
        ob.y - pos_.y) <= std::max(params_.detection_radius, speed_ * params_.t_period))
    {
      objects_.push_back(ob);
    }
  }
}

void FrenetPathPlanner::update_pose(race_msgs::msg::VehicleKinematicState::SharedPtr & pose_msg)
{
  pos_.x = pose_msg->pose.pose.position.x;
  pos_.y = pose_msg->pose.pose.position.y;
  auto q = tf2::Quaternion();
  tf2::fromMsg(pose_msg->pose.pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  yaw_ = y;
  speed_ = pose_msg->speed_mps;
  speed_decomposed_.x = pose_msg->speed_mps * std::cos(yaw_);
  speed_decomposed_.y = pose_msg->speed_mps * std::sin(yaw_);
}

void FrenetPathPlanner::update_ttc(race_msgs::msg::TargetTrajectoryCommand::SharedPtr & ttc_msg)
{
  current_ttl_idx_ = static_cast<race::ttl::TtlIndex>(ttc_msg->current_ttl_index);
  target_speed_ = ttc_msg->target_speed;
  strategy_type_ = ttc_msg->strategy_type.strategy_type;
  area_type_ = ttc_msg->strategy_type.width_type;
  stop_type_ = ttc_msg->stop_type;
  if (ttc_msg->rival_car_exists) {
    rival_car_.exists = true;
    rival_car_.should_follow =
      (ttc_msg->strategy_type.strategy_type == race_msgs::msg::StrategyType::FOLLOW_MODE);
    rival_car_.current_gap = ttc_msg->rival_car_gap;
    rival_car_.target_gap = ttc_msg->target_gap;
    rival_car_.speed = ttc_msg->rival_car_speed;
  }
}
}  // namespace race_path_planner
}  // namespace race
