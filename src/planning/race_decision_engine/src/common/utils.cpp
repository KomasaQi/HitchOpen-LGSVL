// Copyright 2023 Siddharth Saha
// Modified 2024 Moises Lopez

#include "race_decision_engine/common/utils.hpp"

#include <cmath>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

#include "race_decision_engine_parameters.hpp"
#include "ttl_tree.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace common
{
double GetEuclideanNorm(const geometry_msgs::msg::Vector3 & ros_geometry_vector)
{
  return std::hypot(ros_geometry_vector.x, ros_geometry_vector.y);
}
RivalCar GetHighestSpeedRivalCar(
  const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & tracked_objects_msg)
{
  RivalCar rival_car = CreateFalseRivalCar();
  double highest_speed = -1.0;
  double rival_car_speed = 0.0;
  for (const auto & object : tracked_objects_msg->objects) {
    rival_car_speed = GetEuclideanNorm(object.kinematics.twist_with_covariance.twist.linear);
    if (rival_car_speed > highest_speed) {
      highest_speed = rival_car_speed;
      rival_car.pos.x = object.kinematics.pose_with_covariance.pose.position.x;
      rival_car.pos.y = object.kinematics.pose_with_covariance.pose.position.y;
      rival_car.exists = true;
      rival_car.speed = highest_speed;
    }
  }
  return rival_car;
}

RivalCar GetClosestRivalCar(
  const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & tracked_objects_msg,
  const CarState & car_state, const race::ttl::TtlTree::ConstSharedPtr & ttl_tree)
{
  RivalCar rival_car = CreateFalseRivalCar();
  double closest_distance = std::numeric_limits<double>::max();

  auto ego_position = ttl_tree->get_ttl(car_state.current_ttl_index)
    .waypoints.at(car_state.current_ttl_wp_index)
    .location;

  for (const auto & object : tracked_objects_msg->objects) {
    double distance =
      std::hypot(
      object.kinematics.pose_with_covariance.pose.position.x - ego_position.x,
      object.kinematics.pose_with_covariance.pose.position.y - ego_position.y);

    if (distance < closest_distance) {
      closest_distance = distance;
      rival_car.pos.x = object.kinematics.pose_with_covariance.pose.position.x;
      rival_car.pos.y = object.kinematics.pose_with_covariance.pose.position.y;
      rival_car.exists = true;
      rival_car.speed = GetEuclideanNorm(object.kinematics.twist_with_covariance.twist.linear);
    }
  }

  return rival_car;
}

RivalCar GetSecondClosestRivalCar(
  const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & tracked_objects_msg,
  const CarState & car_state, const race::ttl::TtlTree::ConstSharedPtr & ttl_tree)
{
  RivalCar closest_car = CreateFalseRivalCar();
  RivalCar second_closest_car = CreateFalseRivalCar();
  double closest_distance = std::numeric_limits<double>::max();
  double second_closest_distance = std::numeric_limits<double>::max();

  auto ego_position = ttl_tree->get_ttl(car_state.current_ttl_index)
    .waypoints.at(car_state.current_ttl_wp_index)
    .location;

  for (const auto & object : tracked_objects_msg->objects) {
    double distance =
      std::hypot(
      object.kinematics.pose_with_covariance.pose.position.x - ego_position.x,
      object.kinematics.pose_with_covariance.pose.position.y - ego_position.y);

    if (!closest_car.exists) {
      closest_distance = distance;
      closest_car.pos.x = object.kinematics.pose_with_covariance.pose.position.x;
      closest_car.pos.y = object.kinematics.pose_with_covariance.pose.position.y;
      closest_car.exists = true;
      closest_car.speed = GetEuclideanNorm(object.kinematics.twist_with_covariance.twist.linear);
    } else if (distance < closest_distance) {
      second_closest_car.pos.x = closest_car.pos.x;
      second_closest_car.pos.y = closest_car.pos.y;
      second_closest_car.speed = closest_car.speed;
      second_closest_car.exists = true;

      second_closest_distance = closest_distance;
      closest_distance = distance;
      closest_car.pos.x = object.kinematics.pose_with_covariance.pose.position.x;
      closest_car.pos.y = object.kinematics.pose_with_covariance.pose.position.y;
      closest_car.speed = GetEuclideanNorm(object.kinematics.twist_with_covariance.twist.linear);
    } else if (distance < second_closest_distance) {
      second_closest_distance = distance;
      second_closest_car.pos.x = object.kinematics.pose_with_covariance.pose.position.x;
      second_closest_car.pos.y = object.kinematics.pose_with_covariance.pose.position.y;
      second_closest_car.speed =
        GetEuclideanNorm(object.kinematics.twist_with_covariance.twist.linear);
      second_closest_car.exists = true;
    }
  }
  return second_closest_car;
}

RivalCar CreateFalseRivalCar()
{
  RivalCar false_rival_car;
  false_rival_car.pos.x = 0.0;
  false_rival_car.pos.y = 0.0;
  false_rival_car.front_gap = 0.0;
  false_rival_car.back_gap = 0.0;
  false_rival_car.speed = 0.0;
  false_rival_car.exists = false;
  false_rival_car.current_ttl = race::ttl::TtlIndex::INVALID;
  return false_rival_car;
}

void FindRivalCarHalf(
  RivalCar & rival_car, const race::ttl::TtlTree::ConstSharedPtr & ttl_tree,
  const race::ttl::TtlIndex & left_ttl_idx,
  const race::ttl::TtlIndex & right_ttl_idx, double cte_thresh)
{
  rival_car.current_ttl = GetLineNominal(
    rival_car.pos, ttl_tree, cte_thresh, left_ttl_idx,
    right_ttl_idx, rival_car.current_ttl);
}

void FillRivalCarGaps(
  const CarState & car_state, RivalCar & rival_car,
  const race::ttl::TtlTree::ConstSharedPtr & ttl_tree)
{
  double rival_car_dist =
    ttl_tree->get_ttl(car_state.current_ttl_index)
    .waypoints
    .at(ttl_tree->find_closest_waypoint_index(car_state.current_ttl_index, rival_car.pos))
    .dist_to_sf_bwd;
  double gap = rival_car_dist - car_state.lap_distance;
  if (std::abs(gap) > ttl_tree->get_ttl(car_state.current_ttl_index).header.total_distance / 2.0) {
    if (gap > 0.0) {
      gap = gap - ttl_tree->get_ttl(car_state.current_ttl_index).header.total_distance;
    } else {
      gap = ttl_tree->get_ttl(car_state.current_ttl_index).header.total_distance + gap;
    }
  }
  if (gap > 0.0) {
    rival_car.front_gap = gap;
    rival_car.back_gap = ttl_tree->get_ttl(car_state.current_ttl_index).header.total_distance - gap;
  } else {
    rival_car.front_gap =
      ttl_tree->get_ttl(car_state.current_ttl_index).header.total_distance + gap;
    rival_car.back_gap = -gap;
  }
}

bool CheckTimeout(
  bool in_autonomous, double manual_timeout, double autonomous_timeout,
  double actual_timeout)
{
  if (in_autonomous) {
    if (actual_timeout >= autonomous_timeout) {
      return true;
    }
  } else {
    if (actual_timeout >= manual_timeout) {
      return true;
    }
  }
  return false;
}

CarState ConvertLocalizationStateToCarState(
  const race_msgs::msg::VehicleKinematicState::ConstSharedPtr & localization_msg,
  const race::ttl::TtlIndex & current_ttl_index, const race::ttl::TtlIndex & left_ttl_index,
  const race::ttl::TtlIndex & right_ttl_index,
  const race::ttl::TtlIndex & previous_closest_ttl_index,
  const race::ttl::TtlTree::ConstSharedPtr & ttl_tree, double cte_threshold)
{
  CarState car_state{};
  car_state.pos.x = localization_msg->pose.pose.position.x;
  car_state.pos.y = localization_msg->pose.pose.position.y;
  car_state.speed = localization_msg->speed_mps;
  car_state.current_ttl_index = current_ttl_index;
  car_state.lap_distance =
    ttl_tree->get_ttl(current_ttl_index)
    .waypoints.at(ttl_tree->find_closest_waypoint_index(current_ttl_index, car_state.pos))
    .dist_to_sf_bwd;
  car_state.lap_percentage =
    car_state.lap_distance / ttl_tree->get_ttl(current_ttl_index).header.total_distance;
  car_state.lap_percentage *= 100.0;
  car_state.closest_ttl_index =
    GetLineNominal(
    car_state.pos, ttl_tree, cte_threshold, left_ttl_index, right_ttl_index,
    previous_closest_ttl_index);
  car_state.current_ttl_wp_index =
    ttl_tree->find_closest_waypoint_index(car_state.current_ttl_index, car_state.pos);
  car_state.closest_ttl_wp_index =
    ttl_tree->find_closest_waypoint_index(car_state.closest_ttl_index, car_state.pos);
  return car_state;
}

race::ttl::TtlIndex GetLineNominal(
  const race::ttl::Position & pos,
  const race::ttl::TtlTree::ConstSharedPtr & ttl_tree,
  double cte_threshold, const race::ttl::TtlIndex & left_ttl_index,
  const race::ttl::TtlIndex & right_ttl_index,
  const race::ttl::TtlIndex & previous_closest_ttl_index)
{
  race::ttl::TtlIndex closest_ttl_index;
  auto left_cte =
    race::ttl::get_cross_track_error(
    ttl_tree->get_ttl(left_ttl_index), pos,
    ttl_tree->find_closest_waypoint_index(left_ttl_index, pos));
  auto right_cte =
    race::ttl::get_cross_track_error(
    ttl_tree->get_ttl(right_ttl_index), pos,
    ttl_tree->find_closest_waypoint_index(right_ttl_index, pos));
  if (std::abs(left_cte) >= cte_threshold && std::abs(right_cte) >= cte_threshold &&
    previous_closest_ttl_index != race::ttl::TtlIndex::INVALID)
  {
    closest_ttl_index = previous_closest_ttl_index;
  } else if (std::abs(left_cte) > std::abs(right_cte)) {
    closest_ttl_index = right_ttl_index;
  } else {
    closest_ttl_index = left_ttl_index;
  }
  return closest_ttl_index;
}

std::pair<size_t, double> InterpolateSpeed(
  const std::vector<double> & percentages,
  const std::vector<double> & speeds,
  const double & input_percentage)
{
  // Check if the input_percentage is outside the valid range.
  if (input_percentage <= percentages.front()) {
    return std::make_pair(0, speeds.front());
  } else if (input_percentage >= percentages.back()) {
    return std::make_pair(speeds.size() - 1, speeds.front());
  }

  // Perform piecewise linear interpolation.
  for (size_t i = 1; i < percentages.size(); i++) {
    if (input_percentage == percentages[i]) {
      return std::make_pair(i, speeds[i]);
    } else if (input_percentage < percentages[i]) {
      double lower_percentage = percentages[i - 1];
      double upper_percentage = percentages[i];
      double lower_speed = speeds[i - 1];
      double upper_speed = speeds[i];

      // Calculate the fraction input_percentage is between the two neighboring percentages.
      double fraction =
        (input_percentage - lower_percentage) / (upper_percentage - lower_percentage);

      // Perform linear interpolation between the two speeds.
      double final_speed = lower_speed + fraction * (upper_speed - lower_speed);
      return std::make_pair(i, final_speed);
    }
  }
  // The function should never reach this point, but if it does, return 0.0 to stop.
  return std::make_pair(0, 0.0);
}

std::optional<double>
ReturnComparisonGapThreshold(
  const std::shared_ptr<ros_parameters::Params> & params,
  const std::string & thresh_name)
{
  std::optional<double> threshold;
  if (thresh_name == "object_detection") {
    threshold = params->gaps.object_detection;
  } else if (thresh_name == "attacker_preparing") {
    threshold = params->gaps.attacker_preparing;
  } else if (thresh_name == "attacker_attacking") {
    threshold = params->gaps.attacker_attacking;
  } else if (thresh_name == "defender_vicinity") {
    threshold = params->gaps.defender_vicinity;
  } else if (thresh_name == "defender_overtaken") {
    threshold = params->gaps.defender_overtaken;
  } else if (thresh_name == "attacker_done") {
    threshold = params->gaps.attacker_done;
  }
  return threshold;
}
}  // namespace common
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
