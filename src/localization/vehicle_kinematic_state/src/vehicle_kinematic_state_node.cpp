// Copyright 2024 AI Racing Tech

#include "vehicle_kinematic_state/vehicle_kinematic_state_node.hpp"

using std::placeholders::_1;
namespace race
{
VehicleKinematicStateNode::VehicleKinematicStateNode(const rclcpp::NodeOptions & options)
: Node("vehicle_kinematic_state_node", options)
{
  dt_gps_ = this->declare_parameter<double>("gps_publish_s", 0.05);
  dt_imu_ = this->declare_parameter<double>("imu_publish_s", 0.01);
  map_origin_ = {
    declare_parameter("map_origin.latitude", 0.0),
    declare_parameter("map_origin.longitude", 0.0),
    declare_parameter("map_origin.altitude", 0.0)
  };
  this->late_threshold = this->declare_parameter<double>("late_threshold_sec");
  this->num_gps_subscribers_ = this->declare_parameter("num_gps_subscribers", 1);
  this->num_imu_subscribers_ = this->declare_parameter("num_imu_subscribers", 1);
  this->min_lat_thresh_ = this->declare_parameter("min_lat_thresh", 0.35);
  this->min_lon_thresh_ = this->declare_parameter("min_lon_thresh", 0.35);
  this->max_cov0_thresh_ = this->declare_parameter("max_cov0_thresh", 300.0);
  this->max_cov7_thresh_ = this->declare_parameter("max_cov7_thresh", 300.0);
  this->max_jerk_thresh_ = this->declare_parameter<double>("max_jerk_thresh");
  this->max_acc_thresh_ = this->declare_parameter<double>("max_acc_thresh");
  this->max_yaw_rate_ = this->declare_parameter<double>("max_yaw_rate");
  this->publish_odom_ = this->declare_parameter<bool>("publish_odom", true);
  this->drop_period_s_ = this->declare_parameter<double>("drop_period_s", 3.0);
  if (num_gps_subscribers_ < 1 || num_imu_subscribers_ < 1) {
    throw std::runtime_error("Num GPS and IMU subscribers can't be less than one");
  }

  for (int i = 0; i < num_gps_subscribers_; i++) {
    auto & gps_subscriber = this->gps_subscribers_.emplace_back();
    std::string topic_name = "raw_gps_" + std::to_string(i + 1);
    pubsub::subscribe_from(this, gps_subscriber, topic_name);
  }
  for (int i = 0; i < num_imu_subscribers_; i++) {
    auto & imu_subscriber = this->imu_subscribers_.emplace_back();
    std::string topic_name = "raw_imu_" + std::to_string(i + 1);
    pubsub::subscribe_from(this, imu_subscriber, topic_name);
  }

  // Misc initializations
  timer_gps_ = this->create_wall_timer(
    std::chrono::duration<double>(dt_gps_),
    std::bind(&VehicleKinematicStateNode::gps_odom_callback, this));
  timer_imu_ = this->create_wall_timer(
    std::chrono::duration<double>(dt_imu_),
    std::bind(&VehicleKinematicStateNode::imu_odom_callback, this));
  pubsub::publish_to(this, odom_publisher_, "gps_odom", rclcpp::QoS(10));
  pubsub::publish_to(this, state_publisher, "vehicle_kinematic_state", rclcpp::QoS(10));
  pubsub::publish_to(this, emergency_publisher, "car_emergency_state", rclcpp::QoS(10));
  pubsub::publish_to(this, imu_publisher_, "imu", rclcpp::QoS(10));
  pubsub::subscribe_from(this, steering_report_subscriber, "steering_report");
  pubsub::subscribe_from(this, accel_subscriber, "acceleration");
  reset_subscriber = create_subscription<std_msgs::msg::Empty>(
    "reset_vks", rclcpp::QoS{10}, std::bind(
      &VehicleKinematicStateNode::on_reset, this,
      std::placeholders::_1));
  filtered_odom_subscriber = create_subscription<nav_msgs::msg::Odometry>(
    "filtered_odom", rclcpp::QoS{10}, std::bind(
      &VehicleKinematicStateNode::state_callback, this, _1));

  publish_tf_ = this->declare_parameter<bool>("publish_tf", false);
  tf_helper_ = std::make_unique<TransformHelper>(*this);

  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&VehicleKinematicStateNode::parameters_callback, this, _1));

  const double lin_cutoff_freq = declare_parameter<double>("imu_lin_cutoff", 10.0);
  const double ang_cutoff_freq = declare_parameter<double>("imu_ang_cutoff", 10.0);

  drop_start_time_ = now() - rclcpp::Duration::from_seconds(drop_period_s_);
}

/**
 * Return latest gps odometry msg.
 *      If xxx critereas met
 *          raise SOURCE_POS_BAD, and other flags
 *
 */
void VehicleKinematicStateNode::gps_odom_callback()
{
  // Check if all subscribers are publishing
  for (auto & gps_subscriber : gps_subscribers_) {
    if (!gps_subscriber->has_seen_msg()) {
      RCLCPP_ERROR(this->get_logger(), "One GPS not publishing");
      return;
    }
  }
  if (!steering_report_subscriber->has_seen_msg()) {
    RCLCPP_ERROR(this->get_logger(), "Steering report not publishing");
    return;
  }

  // Find a good GPS and IMU subscriber, and check if the car is bad
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_link";
  const auto this_time = rclcpp::Time(odom_msg_.header.stamp);
  auto gps_msg = gps_subscribers_.at(current_best_idx_)->last_received_msg();
  bool gps_bad = (this_time - gps_msg->header.stamp).seconds() > late_threshold ||
    gps_msg->position_covariance[0] > min_lat_thresh_ ||
    gps_msg->position_covariance[4] > min_lon_thresh_;
  bool some_gps_good = !gps_bad;
  if (gps_bad) {
    for (int i = 0; i < num_gps_subscribers_; i++) {
      gps_msg = gps_subscribers_.at(i)->last_received_msg();
      gps_bad = (this_time - gps_msg->header.stamp).seconds() > late_threshold ||
        gps_msg->position_covariance[0] > min_lat_thresh_ ||
        gps_msg->position_covariance[4] > min_lon_thresh_;
      if (!gps_bad) {
        current_best_idx_ = i;
        some_gps_good = true;
        break;
      }
    }
  }

  // Once we have a good GPS and IMU, we can publish the odometry message
  auto latest_gps_msg = gps_subscribers_.at(current_best_idx_)->last_received_msg();
  auto latest_steering_report_msg = steering_report_subscriber->last_received_msg();

  // Steering related stuff
  if (latest_steering_report_msg != nullptr &&
    (this_time - latest_steering_report_msg->stamp).seconds() > late_threshold)
  {
    state_msg.source_status_code |= 1 <<
      race_msgs::msg::VehicleKinematicState::SOURCE_WHEEL_ANGLE_BAD;
  }

  if (latest_steering_report_msg != nullptr) {
    state_msg.front_wheel_angle_rad = latest_steering_report_msg->front_wheel_angle_rad;
  }

  // GPS Related stuff
  if (latest_gps_msg != nullptr) {
    odom_msg_.header.stamp = latest_gps_msg->header.stamp;
    const auto position =
      ttl::GpsPosition {latest_gps_msg->latitude, latest_gps_msg->longitude,
      latest_gps_msg->altitude};
    const auto position3d = ttl::to_enu(position, map_origin_);
    odom_msg_.pose.pose.position.x = position3d.x;
    odom_msg_.pose.pose.position.y = position3d.y;
    odom_msg_.pose.pose.position.z = position3d.z;
    odom_msg_.pose.covariance[0] = latest_gps_msg->position_covariance[0];
    odom_msg_.pose.covariance[7] = latest_gps_msg->position_covariance[4];
    odom_msg_.pose.covariance[14] = latest_gps_msg->position_covariance[8];

    if (abs(latest_gps_msg->speed) < 0.1) {
      latest_gps_msg->speed = 0.0;
    }
    state_msg.speed_mps = static_cast<float>(latest_gps_msg->speed);
    odom_msg_.twist.twist.linear.x = latest_gps_msg->speed;
  }

  // Publish the odometry message if all conditions are met
  if (this->publish_odom_ && some_gps_good && some_imu_good_ &&
    (this_time - drop_start_time_).seconds() > drop_period_s_)
  {
    odom_publisher_->publish(odom_msg_);
  }
}

void VehicleKinematicStateNode::imu_odom_callback()
{
  for (auto & imu_subscriber : imu_subscribers_) {
    if (!imu_subscriber->has_seen_msg()) {
      RCLCPP_ERROR(this->get_logger(), "One IMU not publishing");
      return;
    }
  }

  const auto this_time = rclcpp::Time(odom_msg_.header.stamp);

  auto imu_msg = imu_subscribers_.at(current_best_imu_idx_)->last_received_msg();
  bool imu_bad = (this_time - imu_msg->header.stamp).seconds() > late_threshold &&
    current_best_idx_ != current_best_imu_idx_;  // Don't trust the IMU if the GPS is bad
  some_imu_good_ = !imu_bad;
  if (imu_bad) {
    for (int i = 0; i < num_imu_subscribers_; i++) {
      imu_msg = imu_subscribers_.at(i)->last_received_msg();
      imu_bad = (this_time - imu_msg->header.stamp).seconds() > late_threshold;
      if (!imu_bad) {
        current_best_imu_idx_ = i;
        some_imu_good_ = true;
        break;
      }
    }
  }
  if (!some_imu_good_) {
    state_msg.source_status_code |= 1 << race_msgs::msg::VehicleKinematicState::SOURCE_ACCEL_BAD;
  } else if (!car_bad_) {
    // Check for acceleration constraints
    auto prev_averaged_accel = averaged_accel_;
    std::vector<double> lat_accs;
    std::vector<double> lon_accs;
    for (int i = 0; i < num_imu_subscribers_; i++) {
      imu_bad = (this_time - imu_msg->header.stamp).seconds() > late_threshold;
      if (!imu_bad) {
        lat_accs.push_back(imu_msg->linear_acceleration.x);
        lon_accs.push_back(imu_msg->linear_acceleration.y);
        if (std::abs(imu_msg->angular_velocity.z) > max_yaw_rate_) {
          car_bad_ = true;
          break;
        }
      }
    }
    // Check for jerk constraints
    if (!car_bad_) {
      double mean_lat_acc = std::reduce(lat_accs.begin(), lat_accs.end()) / lat_accs.size();
      double mean_lon_acc = std::reduce(lon_accs.begin(), lon_accs.end()) / lon_accs.size();
      averaged_accel_.linear.x = mean_lat_acc;
      averaged_accel_.linear.y = mean_lon_acc;
      double averaged_jerk = std::hypot(
        mean_lat_acc - prev_averaged_accel.linear.y,
        mean_lon_acc - prev_averaged_accel.linear.x) / dt_gps_;
      for (size_t i = 0; i < lat_accs.size(); i++) {
        bool car_crashed =
          std::hypot(
          lat_accs.at(i),
          lon_accs.at(i)) > max_acc_thresh_ && averaged_jerk > max_jerk_thresh_;
        if (car_crashed) {
          car_bad_ = true;
        }
      }
    }
  }
  if (car_bad_) {
    state_msg.source_status_code |= 1 <<
      race_msgs::msg::VehicleKinematicState::SOURCE_ACCEL_BAD;
    std_msgs::msg::Bool emergency_msg;
    emergency_msg.data = true;
    emergency_publisher->publish(emergency_msg);
  }

  auto latest_imu_msg = imu_subscribers_.at(current_best_imu_idx_)->last_received_msg();

  // IMU Related stuff
  if (latest_imu_msg != nullptr) {
    // Filter orientation (Condition yaw)
    auto orientation = latest_imu_msg->orientation;
    tf2::Quaternion q_tf2;
    tf2::fromMsg(orientation, q_tf2);
    tf2::Matrix3x3 m(q_tf2);
    orientation = tf2::toMsg(q_tf2);
    latest_imu_msg->orientation = orientation;

    // Filter linear acceleration
    latest_imu_msg->linear_acceleration.x = imu_lin_x_filter_.update(
      latest_imu_msg->linear_acceleration.x);
    latest_imu_msg->linear_acceleration.y = imu_lin_y_filter_.update(
      latest_imu_msg->linear_acceleration.y);
    latest_imu_msg->linear_acceleration.z = imu_lin_z_filter_.update(
      latest_imu_msg->linear_acceleration.z);

    // Filter angular velocity
    latest_imu_msg->angular_velocity.x =
      imu_ang_x_filter_.update(latest_imu_msg->angular_velocity.x);
    latest_imu_msg->angular_velocity.y =
      imu_ang_y_filter_.update(latest_imu_msg->angular_velocity.y);
    latest_imu_msg->angular_velocity.z =
      imu_ang_z_filter_.update(latest_imu_msg->angular_velocity.z);

    state_msg.car_yaw = TransformHelper::heading_from_quaternion(orientation);
    imu_publisher_->publish(*latest_imu_msg);

    odom_msg_.pose.pose.orientation = orientation;
    odom_msg_.pose.covariance[21] = latest_imu_msg->orientation_covariance[0];
    odom_msg_.pose.covariance[28] = latest_imu_msg->orientation_covariance[4];
    odom_msg_.pose.covariance[35] = latest_imu_msg->orientation_covariance[8];
    odom_msg_.twist.twist.angular = latest_imu_msg->angular_velocity;
    odom_msg_.twist.covariance[21] = latest_imu_msg->angular_velocity_covariance[0];
    odom_msg_.twist.covariance[28] = latest_imu_msg->angular_velocity_covariance[4];
    odom_msg_.twist.covariance[35] = latest_imu_msg->angular_velocity_covariance[8];

    state_msg.velocity_yaw = TransformHelper::heading_from_quaternion(latest_imu_msg->orientation);
    state_msg.velocity_yaw_rate = latest_imu_msg->angular_velocity.z;
  }
}

void VehicleKinematicStateNode::state_callback(nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  state_msg.header = odom_msg->header;
  state_msg.header.frame_id = "map";
  state_msg.pose = odom_msg->pose;
  state_msg.velocity = odom_msg->twist;

  auto accel_msg = accel_subscriber->last_received_msg();
  if (accel_msg != nullptr) {
    state_msg.accel = accel_msg->accel;
    state_msg.car_yaw_rate = state_msg.velocity.twist.angular.z;
  }

  if (state_msg.pose.covariance[0] > max_cov0_thresh_ ||
    state_msg.pose.covariance[7] > max_cov7_thresh_)
  {
    state_msg.source_status_code |= 1 << race_msgs::msg::VehicleKinematicState::SOURCE_POS_BAD;
    state_msg.source_status_code |= 1 << race_msgs::msg::VehicleKinematicState::SOURCE_SPEED_BAD;
  } else if (state_msg.source_status_code == 3) {
    state_msg.source_status_code = 0;
  }

  state_publisher->publish(state_msg);

  if (publish_tf_) {
    // TODO(moises): Turn into a static transform
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.header.stamp = state_msg.header.stamp;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    tf_helper_->send_transform(t);
  }
}

void VehicleKinematicStateNode::on_reset(std_msgs::msg::Empty::SharedPtr msg)
{
  (void) msg;
  car_bad_ = false;
}

rcl_interfaces::msg::SetParametersResult VehicleKinematicStateNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "No parameters updated";
  result.successful = false;
  RCLCPP_INFO(this->get_logger(), "Publish odom is %d", this->publish_odom_);
  for (const auto & param : parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (param.get_name() == "publish_odom") {
        this->publish_odom_ = static_cast<bool>(param.as_bool());
        RCLCPP_INFO(this->get_logger(), "Publish odom is %d", this->publish_odom_);
        result.successful = true;
        return result;
      } else if (param.get_name() == "publish_tf") {
        this->publish_tf_ = param.as_bool();
        result.successful = true;
        return result;
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (param.get_name() == "imu_lin_cutoff") {
        const double lin_cutoff_freq = param.as_double();
        imu_lin_x_filter_ = LowPassFilter(lin_cutoff_freq, dt_gps_);
        imu_lin_y_filter_ = LowPassFilter(lin_cutoff_freq, dt_gps_);
        imu_lin_z_filter_ = LowPassFilter(lin_cutoff_freq, dt_gps_);
        result.successful = true;
        return result;
      } else if (param.get_name() == "imu_ang_cutoff") {
        const double ang_cutoff_freq = param.as_double();
        imu_ang_x_filter_ = LowPassFilter(ang_cutoff_freq, dt_gps_);
        imu_ang_y_filter_ = LowPassFilter(ang_cutoff_freq, dt_gps_);
        imu_ang_z_filter_ = LowPassFilter(ang_cutoff_freq, dt_gps_);
        result.successful = true;
        return result;
      } else if (param.get_name() == "drop_period_s") {
        drop_period_s_ = param.as_double();
        drop_start_time_ = now();
        result.successful = true;
        return result;
      } else if (param.get_name() == "max_cov0_thresh") {
        max_cov0_thresh_ = param.as_double();
        result.successful = true;
        return result;
      } else if (param.get_name() == "max_cov7_thresh") {
        max_cov7_thresh_ = param.as_double();
        result.successful = true;
        return result;
      }
    }
  }

  return result;
}
}  // namespace race

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  rclcpp::spin(std::make_shared<race::VehicleKinematicStateNode>(options));
  rclcpp::shutdown();
  return 0;
}
