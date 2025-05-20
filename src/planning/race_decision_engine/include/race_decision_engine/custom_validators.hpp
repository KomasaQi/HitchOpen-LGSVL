// Copyright 2023 Siddharth Saha

#ifndef RACE_DECISION_ENGINE__CUSTOM_VALIDATORS_HPP_
#define RACE_DECISION_ENGINE__CUSTOM_VALIDATORS_HPP_

#include <fmt/core.h>

#include <string>
#include <vector>

#include <tl_expected/expected.hpp>

#include "rclcpp/rclcpp.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace custom_validators
{

inline tl::expected<void, std::string> each_element_lower_bound(
  rclcpp::Parameter const & parameter, double min_value
)
{
  const auto & double_array = parameter.as_double_array();
  for (size_t i = 0; i < double_array.size(); i++) {
    if (double_array.at(i) < min_value) {
      return tl::make_unexpected(
        fmt::format(
          "Parameter {} at index {} must be greater than or equal to {}",
          parameter.get_name(), i, min_value));
    }
  }
  return {};
}

inline tl::expected<void, std::string> each_element_upper_bound(
  rclcpp::Parameter const & parameter, double max_value
)
{
  const auto & double_array = parameter.as_double_array();
  for (size_t i = 0; i < double_array.size(); i++) {
    if (double_array.at(i) > max_value) {
      return tl::make_unexpected(
        fmt::format(
          "Parameter {} at index {} must be lesser than or equal to {}",
          parameter.get_name(), i, max_value));
    }
  }
  return {};
}

inline tl::expected<void, std::string> check_array_strictly_increasing(
  rclcpp::Parameter const & parameter)
{
  const auto & double_array = parameter.as_double_array();
  if (double_array.size() >= 2) {
    for (size_t i = 1; i < double_array.size(); i++) {
      if (double_array.at(i) <= double_array.at(i - 1)) {
        return tl::make_unexpected(
          fmt::format(
            "Parameter {} must be in strictly increasing order",
            parameter.get_name()));
      }
    }
  }
  return {};
}

inline tl::expected<void, std::string> check_array_starts_with(
  rclcpp::Parameter const & parameter,
  double start_value)
{
  const auto & double_array = parameter.as_double_array();
  if (!double_array.empty()) {
    if (double_array.front() != start_value) {
      return tl::make_unexpected(
        fmt::format(
          "Parameter {} must start with {}",
          parameter.get_name(), start_value));
    }
  } else {
    return tl::make_unexpected(
      fmt::format(
        "Parameter {} cannot be empty", parameter.get_name(),
        start_value));
  }
  return {};
}

inline tl::expected<void, std::string> check_array_ends_with(
  rclcpp::Parameter const & parameter,
  double end_value)
{
  const auto & double_array = parameter.as_double_array();
  if (!double_array.empty()) {
    if (double_array.back() != end_value) {
      return tl::make_unexpected(
        fmt::format(
          "Parameter {} must end with {}", parameter.get_name(),
          end_value));
    }
  } else {
    return tl::make_unexpected(
      fmt::format(
        "Parameter {} cannot be empty", parameter.get_name(),
        end_value));
  }
  return {};
}
}  // namespace custom_validators
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race

#endif  // RACE_DECISION_ENGINE__CUSTOM_VALIDATORS_HPP_
