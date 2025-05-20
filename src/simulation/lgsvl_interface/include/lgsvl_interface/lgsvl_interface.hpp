// Copyright 2022 Siddharth Saha

#ifndef LGSVL_INTERFACE__LGSVL_INTERFACE_HPP_
#define LGSVL_INTERFACE__LGSVL_INTERFACE_HPP_

#include <string>
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "race_msgs/msg/vehicle_control_command.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"
#include "lgsvl_msgs/msg/can_bus_data.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "lgsvl_msgs/msg/vehicle_odometry.hpp"
#include "race_msgs/msg/steering_report.hpp"
#include "race_msgs/msg/engine_report.hpp"
#include "lgsvl_msgs/msg/bounding_box2_d.hpp"
#include "race_msgs/msg/wheel_speed_report.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "race_msgs/msg/fault_report.hpp"

using lgsvl_msgs::msg::BoundingBox2D;

namespace race
{

class LgsvlInterface : public rclcpp::Node
{
public:
  explicit LgsvlInterface(const rclcpp::NodeOptions & options);

private:
  void step_100_hz();

  void step_20_hz();

  void on_vehicle_control_received(const race_msgs::msg::VehicleControlCommand::SharedPtr msg);

  void on_can_data_received(const lgsvl_msgs::msg::CanBusData::SharedPtr msg);

  void on_wheel_speeds_received(BoundingBox2D::SharedPtr msg);

  void on_vehicle_odom_received(const lgsvl_msgs::msg::VehicleOdometry::SharedPtr msg);

  void on_odom_received(const nav_msgs::msg::Odometry::SharedPtr msg);

  void on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg);

  void on_fix_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg, const int & index);

  void compressed_image_callback(
    sensor_msgs::msg::CompressedImage::SharedPtr msg,
    const int & index);

  rclcpp::Subscription<race_msgs::msg::VehicleControlCommand>::SharedPtr vehicle_control_sub_;
  rclcpp::Subscription<lgsvl_msgs::msg::CanBusData>::SharedPtr can_bus_sub_;
  rclcpp::Subscription<lgsvl_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;
  rclcpp::Subscription<lgsvl_msgs::msg::BoundingBox2D>::SharedPtr wheel_speeds_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;

  bool can_seen_ {false};
  bool vehicle_odom_seen_ {false};
  bool odom_seen_ {false};
  bool imu_seen_ {false};
  bool fix_seen_ {false};

  rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr lgsvl_control_pub_;
  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_pub_;
  rclcpp::Publisher<race_msgs::msg::SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<race_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speeds_pub;
  rclcpp::Publisher<race_msgs::msg::EngineReport>::SharedPtr engine_pub_;
  rclcpp::Publisher<race_msgs::msg::FaultReport>::SharedPtr low_level_fault_report_pub_;

  race_msgs::msg::VehicleControlCommand::SharedPtr vehicle_control_msg_;
  lgsvl_msgs::msg::CanBusData::SharedPtr can_bus_msg_;
  lgsvl_msgs::msg::VehicleOdometry::SharedPtr vehicle_odom_msg_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_;
  sensor_msgs::msg::Imu::SharedPtr imu_msg_;
  sensor_msgs::msg::NavSatFix::SharedPtr fix_msg_;

  lgsvl_msgs::msg::VehicleControlData lgsvl_control_msg_;
  race_msgs::msg::SteeringReport steering_msg_;
  race_msgs::msg::WheelSpeedReport wheel_speeds_report_msg_;
  race_msgs::msg::EngineReport engine_msg_;
  gps_msgs::msg::GPSFix gps_fix_msg_;

  rclcpp::TimerBase::SharedPtr step_timer_100_hz_;
  rclcpp::TimerBase::SharedPtr step_timer_20_hz_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pc_subscriptions_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pc_publishers_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr>
  image_subscriptions_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_publishers_;
  std::vector<cv_bridge::CvImagePtr> cv_ptrs_;
};

}  // namespace race

#endif  // LGSVL_INTERFACE__LGSVL_INTERFACE_HPP_
