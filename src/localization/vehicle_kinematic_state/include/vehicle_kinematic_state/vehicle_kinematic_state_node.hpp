// Copyright 2024 AI Racing Tech

#ifndef VEHICLE_KINEMATIC_STATE__VEHICLE_KINEMATIC_STATE_NODE_HPP_
#define VEHICLE_KINEMATIC_STATE__VEHICLE_KINEMATIC_STATE_NODE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ttl.hpp"
#include "base_common/pubsub.hpp"
#include "base_common/low_pass_filter.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "race_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "transform_helper/transform_helper.hpp"
#include "std_msgs/msg/empty.hpp"

namespace race
{
class VehicleKinematicStateNode : public rclcpp::Node
{
public:
  explicit VehicleKinematicStateNode(const rclcpp::NodeOptions & options);

private:
  void gps_odom_callback();  // publish gps odom msg
  void imu_odom_callback();  // publish imu odom msg
  void state_callback(nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void on_reset(std_msgs::msg::Empty::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::TimerBase::SharedPtr timer_gps_;
  rclcpp::TimerBase::SharedPtr timer_imu_;

  int num_gps_subscribers_ = 0;
  int num_imu_subscribers_ = 0;
  int current_best_idx_ = 0;
  int current_best_imu_idx_ = 0;

  bool car_bad_ = false;
  bool publish_tf_ = false;
  bool publish_odom_{true};

  bool some_imu_good_ = false;

  std::unique_ptr<TransformHelper> tf_helper_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  geometry_msgs::msg::Accel averaged_accel_;
  std::vector<pubsub::MsgSubscriber<gps_msgs::msg::GPSFix>::SharedPtr> gps_subscribers_;
  std::vector<pubsub::MsgSubscriber<sensor_msgs::msg::Imu>::SharedPtr> imu_subscribers_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<race_msgs::msg::VehicleKinematicState>::SharedPtr state_publisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  pubsub::MsgSubscriber<race_msgs::msg::SteeringReport>::UniquePtr steering_report_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_subscriber;
  pubsub::MsgSubscriber<geometry_msgs::msg::AccelWithCovarianceStamped>::UniquePtr accel_subscriber;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscriber;

  double dt_gps_ = 0.0;
  double dt_imu_ = 0.0;
  double late_threshold = 0.0;
  double min_lat_thresh_ = 0.0;
  double min_lon_thresh_ = 0.0;
  double max_cov0_thresh_ = 0.0;
  double max_cov7_thresh_ = 0.0;
  double max_jerk_thresh_ = 0.0;
  double max_acc_thresh_ = 0.0;
  double max_yaw_rate_ = 0.0;

  double drop_period_s_ = 0.0;
  rclcpp::Time drop_start_time_;

  LowPassFilter imu_lin_x_filter_;
  LowPassFilter imu_lin_y_filter_;
  LowPassFilter imu_lin_z_filter_;
  LowPassFilter imu_ang_x_filter_;
  LowPassFilter imu_ang_y_filter_;
  LowPassFilter imu_ang_z_filter_;

  nav_msgs::msg::Odometry odom_msg_;
  race_msgs::msg::VehicleKinematicState state_msg;

  ttl::GpsPosition map_origin_;
};
}  // namespace race

#endif  // VEHICLE_KINEMATIC_STATE__VEHICLE_KINEMATIC_STATE_NODE_HPP_
