// Copyright 2024 AI Racing Tech

#ifndef ODOMETRY_STATE_ESTIMATOR__ODOMETRY_STATE_ESTIMATOR_NODE_HPP_
#define ODOMETRY_STATE_ESTIMATOR__ODOMETRY_STATE_ESTIMATOR_NODE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/wheel_speed_report.hpp"
#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_msgs/msg/steering_report.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "base_common/pubsub.hpp"
#include "eigen3/Eigen/Dense"
#include "transform_helper/transform_helper.hpp"
#include "base_common/low_pass_filter.hpp"
#include "ttl.hpp"
#include "ttl_tree.hpp"

namespace race::localization::odometry_state_estimator
{
class OdometryStateEstimatorNode : public rclcpp::Node
{
public:
  explicit OdometryStateEstimatorNode(const rclcpp::NodeOptions & options);

private:
  void publish_odom();
  void correct_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

  pubsub::MsgSubscriber<race_msgs::msg::WheelSpeedReport>::UniquePtr wheel_speed_report_subscriber_;
  pubsub::MsgSubscriber<sensor_msgs::msg::Imu>::UniquePtr imu_subscriber_;
  pubsub::MsgSubscriber<race_msgs::msg::TargetTrajectoryCommand>::UniquePtr ttc_subscriber_;
  pubsub::MsgSubscriber<race_msgs::msg::SteeringReport>::UniquePtr steering_report_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_correction_subscriber_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();

  double dt_ = 1.0;
  double wheel_min_calibration_speed_;
  double lin_cutoff_freq_;
  int collection_counter = 1;
  int calibration_counter = 1;
  int collection_frequency;
  int calibration_frequency;
  size_t wheel_calibration_sample_num_;
  bool wheel_calibrated_ = false;
  bool ready_to_publish_ = false;
  bool publish_odom_;
  std::vector<std::vector<double>> wheel_speeds_;
  std::vector<double> gps_speeds_;
  Eigen::Matrix<double, 4, 1> current_wheel_speeds_;

  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::string ttl_dir_;
  race::ttl::TtlIndex current_ttl_idx_;
  race::ttl::TtlTree::SharedPtr ttl_tree_{};

  Eigen::Matrix<double, 4, 1> wheel_correction_factors_;

  LowPassFilter wheel_odom_lin_x_filter_;
};

}  // namespace race::localization::odometry_state_estimator

#endif  // ODOMETRY_STATE_ESTIMATOR__ODOMETRY_STATE_ESTIMATOR_NODE_HPP_
