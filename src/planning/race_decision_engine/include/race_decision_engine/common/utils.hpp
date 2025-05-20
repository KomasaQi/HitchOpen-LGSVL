// Copyright 2023 Siddharth Saha
// Modified 2024 Moises Lopez

#ifndef RACE_DECISION_ENGINE__COMMON__UTILS_HPP_
#define RACE_DECISION_ENGINE__COMMON__UTILS_HPP_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "race_decision_engine/common/input.hpp"
#include "race_decision_engine_parameters.hpp"

#include "ttl_tree.hpp"

#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace race::planning::race_decision_engine::common
{
const double MPH_TO_MPS = 0.44704;
const double DEFAULT_SCALE_FACTOR = 100.0;
const double MAX_LAP_PERCENTAGE = 100.0;
double GetEuclideanNorm(const geometry_msgs::msg::Vector3::ConstSharedPtr & ros_geometry_vector);
RivalCar GetHighestSpeedRivalCar(
  const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & tracked_objects_msg);
RivalCar GetClosestRivalCar(
  const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & tracked_objects_msg,
  const CarState & car_state, const race::ttl::TtlTree::ConstSharedPtr & ttl_tree);
RivalCar GetSecondClosestRivalCar(
  const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr & tracked_objects_msg,
  const CarState & car_state, const race::ttl::TtlTree::ConstSharedPtr & ttl_tree);
RivalCar CreateFalseRivalCar();
void FindRivalCarHalf(
  RivalCar & rival_car, const race::ttl::TtlTree::ConstSharedPtr & ttl_tree,
  const race::ttl::TtlIndex & left_ttl_idx,
  const race::ttl::TtlIndex & right_ttl_idx, double cte_thresh);
void FillRivalCarGaps(
  const CarState & car_state, RivalCar & rival_car,
  const race::ttl::TtlTree::ConstSharedPtr & ttl_tree);
bool CheckTimeout(
  bool in_autonomous, double manual_timeout, double autonomous_timeout,
  double actual_timeout);
CarState ConvertLocalizationStateToCarState(
  const race_msgs::msg::VehicleKinematicState::ConstSharedPtr & localization_msg,
  const race::ttl::TtlIndex & current_ttl_index, const race::ttl::TtlIndex & left_ttl_index,
  const race::ttl::TtlIndex & right_ttl_index,
  const race::ttl::TtlIndex & previous_closest_ttl_index,
  const race::ttl::TtlTree::ConstSharedPtr & ttl_tree, double cte_threshold);
race::ttl::TtlIndex GetLineNominal(
  const race::ttl::Position & pos,
  const race::ttl::TtlTree::ConstSharedPtr & ttl_tree,
  double cte_threshold, const race::ttl::TtlIndex & left_ttl_index,
  const race::ttl::TtlIndex & right_ttl_index,
  const race::ttl::TtlIndex & previous_closest_ttl_index);
std::pair<size_t, double> InterpolateSpeed(
  const std::vector<double> & percentages,
  const std::vector<double> & speeds,
  const double & input_percentage);
std::optional<double>
ReturnComparisonGapThreshold(
  const std::shared_ptr<ros_parameters::Params> & params,
  const std::string & thresh_name);

}  // namespace race::planning::race_decision_engine::common

#endif  // RACE_DECISION_ENGINE__COMMON__UTILS_HPP_
