// Copyright 2024 AI Racing Tech

#include "odometry_state_estimator/odometry_state_estimator_node.hpp"

namespace race::localization::odometry_state_estimator
{
OdometryStateEstimatorNode::OdometryStateEstimatorNode(const rclcpp::NodeOptions & options)
: Node("vehicle_odometry_estimation_node", options)
{
  dt_ = this->declare_parameter<double>("pub_interval_sec", 0.05);

  pubsub::subscribe_from(this, wheel_speed_report_subscriber_, "wheel_speed_report");
  pubsub::subscribe_from(this, imu_subscriber_, "imu");
  pubsub::subscribe_from(this, ttc_subscriber_, "target_trajectory_command");
  pubsub::subscribe_from(this, steering_report_subscriber_, "steering_report");

  pubsub::publish_to(this, odom_publisher_, "odom", rclcpp::QoS(10));

  odom_correction_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom_correction", rclcpp::SensorDataQoS(),
    std::bind(&OdometryStateEstimatorNode::correct_odom, this, std::placeholders::_1));

  odom_msg_->header.frame_id = "odom";
  odom_msg_->child_frame_id = "base_link";

  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<float>(dt_), [this] {
      OdometryStateEstimatorNode::publish_odom();
    });

  this->collection_frequency = this->declare_parameter<int>("collection_frequency", 5);
  this->calibration_frequency = this->declare_parameter<int>("calibration_frequency", 100);

  this->wheel_correction_factors_ << this->declare_parameter<double>(
    "wheel_initial_corrections.front_left", 1.0),
    this->declare_parameter<double>("wheel_initial_corrections.front_right", 1.0),
    this->declare_parameter<double>("wheel_initial_corrections.rear_left", 1.0),
    this->declare_parameter<double>("wheel_initial_corrections.rear_right", 1.0);

  this->wheel_min_calibration_speed_ = this->declare_parameter<double>(
    "wheel_min_calibration_speed_", 10.0);
  this->wheel_calibration_sample_num_ = this->declare_parameter<int>(
    "wheel_calibration_sample_num",
    10.0);
  this->wheel_calibrated_ = this->declare_parameter<bool>("wheel_calibrated", false);
  this->lin_cutoff_freq_ = this->declare_parameter<double>("lin_cutoff_freq", 10.0);
  this->ttl_dir_ = this->declare_parameter<std::string>("ttl_dir", "");

  this->wheel_odom_lin_x_filter_ = LowPassFilter(lin_cutoff_freq_, dt_);
  this->publish_odom_ = this->declare_parameter<bool>("publish_odom", true);

  ttl_tree_ = std::make_unique<race::ttl::TtlTree>(ttl_dir_.c_str());

  ready_to_publish_ = false;

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&OdometryStateEstimatorNode::param_callback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult OdometryStateEstimatorNode::param_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const auto & param : parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (param.get_name() == "lin_cutoff_freq") {
        lin_cutoff_freq_ = param.as_double();
        wheel_odom_lin_x_filter_ = LowPassFilter(lin_cutoff_freq_, dt_);
        result.successful = true;
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (param.get_name() == "publish_odom") {
        publish_odom_ = param.as_bool();
        result.successful = true;
      }
    }
  }
}

void OdometryStateEstimatorNode::publish_odom()
{
  if (!wheel_speed_report_subscriber_->has_seen_msg()) {
    RCLCPP_ERROR(this->get_logger(), "Wheel speed report not publishing");
    return;
  }
  if (!imu_subscriber_->has_seen_msg()) {
    RCLCPP_ERROR(this->get_logger(), "IMU not publishing");
    return;
  }

  if (wheel_speed_report_subscriber_->last_received_msg() == nullptr ||
    imu_subscriber_->last_received_msg() == nullptr)
  {
    RCLCPP_ERROR(this->get_logger(), "Wheel speed report or IMU message is null");
    return;
  }

  // Wheel speed report is used to determine the linear velocity of the vehicle
  auto wheel_speed_report_msg = wheel_speed_report_subscriber_->last_received_msg();
  odom_msg_->header.stamp = wheel_speed_report_msg->header.stamp;
  Eigen::Matrix<double, 4, 1> wheel_speeds;
  wheel_speeds << wheel_speed_report_msg->front_left, wheel_speed_report_msg->front_right,
    wheel_speed_report_msg->rear_left, wheel_speed_report_msg->rear_right;
  current_wheel_speeds_ = wheel_speeds;

  if (!(publish_odom_ && ready_to_publish_)) {
    return;
  }

  Eigen::Matrix<double, 4, 1> corrected_wheel_speeds = wheel_speeds.cwiseProduct(
    this->wheel_correction_factors_);
  double v =
    (corrected_wheel_speeds(0) + corrected_wheel_speeds(1) + corrected_wheel_speeds(2) +
    corrected_wheel_speeds(3)) / 4.0;

  // Fill in information for the odometry message
  odom_msg_->header.stamp = wheel_speed_report_msg->header.stamp;
  odom_msg_->twist.twist.angular.z = imu_subscriber_->last_received_msg()->angular_velocity.z;
  odom_msg_->twist.covariance[35] =
    imu_subscriber_->last_received_msg()->angular_velocity_covariance[8];
  odom_msg_->twist.twist.linear.x = wheel_odom_lin_x_filter_.update(v);
  // odom_msg_->twist.covariance[0] = unknown; // Wheel speed covariance

  odom_publisher_->publish(*odom_msg_);
}

void OdometryStateEstimatorNode::correct_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (msg == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Odometry correction message is null");
    return;
  }

  if (this->wheel_calibrated_) {
    if (this->calibration_counter != this->calibration_frequency) {
      RCLCPP_DEBUG(
        this->get_logger(), "Wheel speed already calibrated %i", this->calibration_counter);
      this->calibration_counter++;
      return;
    } else {
      this->calibration_counter = 1;
      this->wheel_calibrated_ = false;
      RCLCPP_DEBUG(this->get_logger(), "recalibrating wheels");
    }
  }

  auto ttc_msg = ttc_subscriber_->last_received_msg();
  if (ttc_msg == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Target trajectory command message is null");
    return;
  }

  auto steering_report_msg = steering_report_subscriber_->last_received_msg();
  if (steering_report_msg == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Steering report message is null");
    return;
  }

  // Check if we are at a sufficient speed to calibrate the wheel speeds
  for (size_t i = 0; i < 4; i++) {
    if (this->current_wheel_speeds_(i) <= this->wheel_min_calibration_speed_) {
      return;
    }
  }

  double gps_speed = msg->twist.twist.linear.x;
  if (gps_speed <= wheel_min_calibration_speed_) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 5000, "Speed is too low to calibrate wheel speeds");
    return;
  }

  if (ttl_tree_->is_ttl_index_valid(ttc_msg->current_ttl_index)) {
    current_ttl_idx_ = static_cast<race::ttl::TtlIndex>(ttc_msg->current_ttl_index);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid TTL index");
    return;
  }

  race::ttl::Position pos_;
  pos_.x = msg->pose.pose.position.x;
  pos_.y = msg->pose.pose.position.y;

  auto current_ttl_wp_idx = ttl_tree_->find_closest_waypoint_index(current_ttl_idx_, pos_);
  auto region = ttl_tree_->get_ttl(current_ttl_idx_).waypoints.at(current_ttl_wp_idx).region;
  auto at_straight = region == race::ttl::TrackLocation::STRAIGHT;

  if (!at_straight) {
    return;  // Only calibrate on straight sections
  }

  if (this->wheel_speeds_.size() < this->wheel_calibration_sample_num_) {
    if (this->collection_counter == this->collection_frequency) {
      this->collection_counter = 1;
      this->wheel_speeds_.push_back(
        {this->current_wheel_speeds_(0), this->current_wheel_speeds_(
            1), this->current_wheel_speeds_(2), this->current_wheel_speeds_(3)});
      this->gps_speeds_.push_back(gps_speed);
      RCLCPP_DEBUG(
        this->get_logger(), "%ld: Calibrating wheel speed with GPS speed: %f",
        this->wheel_speeds_.size(), gps_speed);
      return;
    } else {
      RCLCPP_DEBUG(
        this->get_logger(), "waiting to collect data at counter %i", this->collection_counter);
      this->collection_counter++;
      return;
    }
  } else {
    RCLCPP_DEBUG(
      this->get_logger(), "Calibrating wheels to gps speed, collected data size: %i, %i",
      this->wheel_speeds_.size(), this->gps_speeds_.size());
    Eigen::MatrixXd wheel_speeds_matrix(this->wheel_calibration_sample_num_, 4);
    Eigen::MatrixXd gps_speed_matrix(this->wheel_calibration_sample_num_, 4);
    Eigen::VectorXd gps_vector(this->wheel_speeds_.size());

    for (size_t i = 0; i < this->wheel_calibration_sample_num_; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        wheel_speeds_matrix(i, j) = this->wheel_speeds_[i][j];
        gps_speed_matrix(i, j) = this->gps_speeds_[i];
      }
    }

    Eigen::MatrixXd ptdiv = gps_speed_matrix.array() / wheel_speeds_matrix.array();
    this->wheel_correction_factors_ = ptdiv.colwise().mean();
    RCLCPP_INFO(
      this->get_logger(), "Calibrating wheel correction: %f, %f, %f, %f",
      this->wheel_correction_factors_(
        0), this->wheel_correction_factors_(1), this->wheel_correction_factors_(
        2), this->wheel_correction_factors_(3));
    this->wheel_calibrated_ = true;
    this->ready_to_publish_ = true;

    this->wheel_speeds_.clear();
    this->gps_speeds_.clear();
    RCLCPP_DEBUG(
      this->get_logger(), "Cleaning up memory: %i, %i",
      this->wheel_speeds_.size(), this->gps_speeds_.size());
    return;
  }
}

}  // namespace race::localization::odometry_state_estimator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  rclcpp::spin(
    std::make_shared<race::localization::odometry_state_estimator::OdometryStateEstimatorNode>(
      options));
  rclcpp::shutdown();
  return 0;
}
