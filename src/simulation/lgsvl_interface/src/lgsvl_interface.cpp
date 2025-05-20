// Copyright 2022 Siddharth Saha

#include <memory>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "lgsvl_interface/lgsvl_interface.hpp"
#include "race_msgs/msg/stop_type.hpp"

namespace race
{
LgsvlInterface::LgsvlInterface(const rclcpp::NodeOptions & options)
: rclcpp::Node("lgsvl_interface", options)
{
  this->declare_parameter<double>("origin.altitude");
  this->declare_parameter<double>("origin.latitude");
  this->declare_parameter<double>("origin.longitude");
  this->declare_parameter<uint8_t>("fault_report_level", 0);
  this->declare_parameter<bool>("publish_fault_report", true);
  this->declare_parameter<std::string>("control_frame");
  this->declare_parameter<std::string>("gps_frame");

  vehicle_control_sub_ = this->create_subscription<race_msgs::msg::VehicleControlCommand>(
    "vehicle_control_cmd", rclcpp::SensorDataQoS(),
    std::bind(&LgsvlInterface::on_vehicle_control_received, this, std::placeholders::_1));
  can_bus_sub_ = this->create_subscription<lgsvl_msgs::msg::CanBusData>(
    "can_bus",
    rclcpp::SensorDataQoS(),
    std::bind(&LgsvlInterface::on_can_data_received, this, std::placeholders::_1));
  vehicle_odom_sub_ = this->create_subscription<lgsvl_msgs::msg::VehicleOdometry>(
    "lgsvl_odom",
    rclcpp::SensorDataQoS(),
    std::bind(&LgsvlInterface::on_vehicle_odom_received, this, std::placeholders::_1));
  wheel_speeds_sub_ = this->create_subscription<BoundingBox2D>(
    "lgsvl/wheel_speeds",
    rclcpp::SensorDataQoS(),
    std::bind(&LgsvlInterface::on_wheel_speeds_received, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu",
    rclcpp::SensorDataQoS(),
    std::bind(&LgsvlInterface::on_imu_received, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    rclcpp::SensorDataQoS(),
    std::bind(&LgsvlInterface::on_odom_received, this, std::placeholders::_1));
  fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "fix",
    rclcpp::SensorDataQoS(),
    std::bind(&LgsvlInterface::on_fix_received, this, std::placeholders::_1));

  lgsvl_control_pub_ = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>(
    "lgsvl_control",
    10);
  gps_fix_pub_ = this->create_publisher<gps_msgs::msg::GPSFix>("gps_fix", rclcpp::SensorDataQoS());
  wheel_speeds_pub = this->create_publisher<race_msgs::msg::WheelSpeedReport>(
    "wheel_speeds",
    rclcpp::SensorDataQoS());

  steering_pub_ = this->create_publisher<race_msgs::msg::SteeringReport>(
    "steering_report",
    rclcpp::SensorDataQoS());
  engine_pub_ = create_publisher<race_msgs::msg::EngineReport>(
    "engine_report",
    rclcpp::SensorDataQoS());
  low_level_fault_report_pub_ = create_publisher<race_msgs::msg::FaultReport>(
    "low_level_fault_report",
    rclcpp::SensorDataQoS());

  step_timer_100_hz_ =
    rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(0.01), [this] {
      step_100_hz();
    });
  step_timer_20_hz_ =
    rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(0.05), [this] {
      step_20_hz();
    });

  // Get the number of pointclouds to subscribe to and publish from the ROS2 parameter server
  int num_pointclouds = this->declare_parameter<int64_t>("num_pointclouds", 0);
  int num_cameras = this->declare_parameter<int64_t>("num_cameras", 0);

  // Create reliable subscriptions to the pointcloud topics
  for (int i = 0; i < num_pointclouds; i++) {
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> cb = std::bind(
      &LgsvlInterface::pointcloud_callback, this,
      std::placeholders::_1, i);
    std::string topic_name = "pointcloud_in_" + std::to_string(i);
    pc_subscriptions_.push_back(
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, rclcpp::QoS{5}, cb));
  }

  // Create best effort publishers for the pointcloud topics
  for (int i = 0; i < num_pointclouds; i++) {
    std::string topic_name = "pointcloud_out_" + std::to_string(i);
    pc_publishers_.push_back(
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_name, rclcpp::SensorDataQoS()));
  }

  // Create reliable subscriptions to the compressed image topics
  for (int i = 0; i < num_cameras; i++) {
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> cb = std::bind(
      &LgsvlInterface::compressed_image_callback, this,
      std::placeholders::_1, i);
    std::string topic_name = "compressed_image_in_" + std::to_string(i);
    image_subscriptions_.push_back(
      this->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic_name, rclcpp::QoS{5}, cb));
  }

  // Create best effort publishers for the de-compressed image topics
  for (int i = 0; i < num_cameras; i++) {
    std::string topic_name = "image_out_" + std::to_string(i);
    image_publishers_.push_back(
      this->create_publisher<sensor_msgs::msg::Image>(
        topic_name, rclcpp::SensorDataQoS()));

    // pre-allocate the CvImagePtr objects
    cv_ptrs_.push_back(std::make_shared<cv_bridge::CvImage>());
  }
}

void LgsvlInterface::pointcloud_callback(
  sensor_msgs::msg::PointCloud2::SharedPtr msg,
  const int & index)
{
  // Publish the received pointcloud with the best effort QoS
  pc_publishers_[index]->publish(*msg);
}

void LgsvlInterface::compressed_image_callback(
  sensor_msgs::msg::CompressedImage::SharedPtr msg,
  const int & index)
{
  // decompress the image and publish it with the best effort QoS
  cv_ptrs_[index] = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  sensor_msgs::msg::Image image = *cv_ptrs_[index]->toImageMsg();
  image_publishers_[index]->publish(image);
}

void LgsvlInterface::on_wheel_speeds_received(BoundingBox2D::SharedPtr msg)
{
  auto now_time = this->now();
  wheel_speeds_report_msg_.header.stamp = now_time;
  wheel_speeds_report_msg_.front_left = msg->x;
  wheel_speeds_report_msg_.front_right = msg->y;
  wheel_speeds_report_msg_.rear_left = msg->width;
  wheel_speeds_report_msg_.rear_right = msg->height;
  wheel_speeds_pub->publish(wheel_speeds_report_msg_);
}

void LgsvlInterface::on_vehicle_control_received(
  race_msgs::msg::VehicleControlCommand::SharedPtr msg)
{
  vehicle_control_msg_ = msg;
  lgsvl_control_msg_.header.stamp = msg->stamp;
  lgsvl_control_msg_.header.frame_id = this->get_parameter("control_frame").as_string();
  lgsvl_control_msg_.acceleration_pct = msg->accelerator_cmd / 100.0;
  static constexpr auto MAX_BRAKE_PSI = 10000.0;
  lgsvl_control_msg_.braking_pct = msg->brake_cmd / MAX_BRAKE_PSI;
  lgsvl_control_msg_.target_wheel_angle = -1.0 * msg->steering_cmd;
  lgsvl_control_msg_.target_gear = msg->gear_cmd;
  lgsvl_control_pub_->publish(lgsvl_control_msg_);
}

void LgsvlInterface::on_can_data_received(const lgsvl_msgs::msg::CanBusData::SharedPtr msg)
{
  can_bus_msg_ = msg;
  can_seen_ = true;
}

void LgsvlInterface::on_vehicle_odom_received(const lgsvl_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  vehicle_odom_msg_ = msg;
  vehicle_odom_seen_ = true;
}

void LgsvlInterface::on_odom_received(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_msg_ = msg;
  odom_seen_ = true;
}

void LgsvlInterface::on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_msg_ = msg;
  imu_seen_ = true;
}

void LgsvlInterface::on_fix_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  fix_msg_ = msg;
  fix_seen_ = true;
}

void LgsvlInterface::step_100_hz()
{
  race_msgs::msg::FaultReport low_level_fault_msg;
  low_level_fault_msg.stamp = now();
  low_level_fault_msg.stop_type.stop_type = this->get_parameter("fault_report_level").as_int();
  low_level_fault_msg.reason = "OSSDC DOES NOT PROVIDE FAULTS";
  if (this->get_parameter("publish_fault_report").as_bool()) {
    low_level_fault_report_pub_->publish(low_level_fault_msg);
  }
  if (imu_seen_) {
    tf2::Quaternion q_tf2;
    tf2::fromMsg(imu_msg_->orientation, q_tf2);
    tf2::Matrix3x3 m(q_tf2);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    gps_fix_msg_.pitch = pitch;
    gps_fix_msg_.roll = roll;
    gps_fix_msg_.track = yaw;
  }
}

void LgsvlInterface::step_20_hz()
{
  if (imu_seen_ && vehicle_odom_seen_ && can_seen_ && odom_seen_ && fix_seen_) {
    auto now_time = this->now();
    steering_msg_.stamp = now_time;
    steering_msg_.front_wheel_angle_rad = -1.0 * vehicle_odom_msg_->front_wheel_angle;
    steering_msg_.front_wheel_angle_rad_cmd = -1.0 * lgsvl_control_msg_.target_wheel_angle;
    steering_pub_->publish(steering_msg_);

    gps_fix_msg_.header.frame_id = this->get_parameter("gps_frame").as_string();
    gps_fix_msg_.header.stamp = now_time;
    gps_fix_msg_.latitude = fix_msg_->latitude;
    gps_fix_msg_.longitude = fix_msg_->longitude;
    gps_fix_msg_.altitude = fix_msg_->altitude;
    gps_fix_msg_.speed = can_bus_msg_->speed_mps;
    gps_fix_pub_->publish(gps_fix_msg_);

    engine_msg_.stamp = now_time;
    engine_msg_.current_gear = can_bus_msg_->selected_gear;
    if (engine_msg_.current_gear == 0) {
      engine_msg_.current_gear = 1;
    }
    engine_msg_.engine_rpm = can_bus_msg_->engine_rpm;
    const auto gear_cmd = vehicle_control_msg_ ? vehicle_control_msg_->gear_cmd : 1;
    const auto gear_diff = gear_cmd - can_bus_msg_->selected_gear;
    if (gear_diff == 0) {
      engine_msg_.gear_shift_status = 1U;
    } else if (gear_diff == 1) {
      engine_msg_.gear_shift_status = 3U;
    } else if (gear_diff == -1) {
      engine_msg_.gear_shift_status = 4U;
    } else {
      engine_msg_.gear_shift_status = 1U;
    }
    engine_pub_->publish(engine_msg_);
  }
}
}  // namespace race

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<race::LgsvlInterface>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
