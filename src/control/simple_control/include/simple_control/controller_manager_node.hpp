#ifndef SIMPLE_CONTROL__CONTROLLER_MANAGER_HPP_
#define SIMPLE_CONTROL__CONTROLLER_MANAGER_HPP_
#include <memory>
#include <vector>
#include <string>
#include "simple_control/controller_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lgsvl_msgs/msg/can_bus_data.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "lgsvl_msgs/msg/bounding_box2_d.hpp"
#include <iostream>
#include <fstream>
#include "race_msgs/msg/vehicle_flag.hpp"

namespace control {

    class ControllerManagerNode: public rclcpp::Node {
        public:
            ControllerManagerNode();
            void addController(const std::string& name, const std::string& type);
            void runStep();

            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr should_publish_control_sub_;
            void should_publish_control_callback(const std_msgs::msg::Bool::SharedPtr msg);
            bool should_publish_control_ = true;

            rclcpp::Subscription<race_msgs::msg::VehicleFlag>::SharedPtr vehicle_flag_sub_;
            void vehicle_flag_callback(const race_msgs::msg::VehicleFlag::SharedPtr msg);
            uint8_t vehicle_flag_ = race_msgs::msg::VehicleFlag::RED;

            rclcpp::Subscription<lgsvl_msgs::msg::BoundingBox2D>::SharedPtr wheel_speeds_sub_;
            void on_wheel_speeds_callback(const lgsvl_msgs::msg::BoundingBox2D::SharedPtr msg);
            lgsvl_msgs::msg::BoundingBox2D::SharedPtr wheel_speeds_;

            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gear_pub_;
            


            rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_pub_; 
            rclcpp::TimerBase::SharedPtr control_timer_;

            rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr local_path_sub_;
            void on_local_path_callback(const autoware_planning_msgs::msg::Path::SharedPtr msg);
            autoware_planning_msgs::msg::Path::SharedPtr local_path_;

            rclcpp::Subscription<lgsvl_msgs::msg::CanBusData>::SharedPtr can_bus_data_sub_;
            void on_can_bus_data_callback(const lgsvl_msgs::msg::CanBusData::SharedPtr msg);
            lgsvl_msgs::msg::CanBusData::SharedPtr can_bus_data_;

            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
            void on_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
            sensor_msgs::msg::Imu::SharedPtr imu_;           
            
            
            typedef std::vector<control::ControllerInterface::SharedPtr> PluginList;
            pluginlib::ClassLoader<control::ControllerInterface> m_plugin_loader_;
            PluginList m_plugins_;


            rclcpp::TimerBase::SharedPtr flag_check_timer_;
            double flag_timeout_{1.0};  // Default timeout of 1 second
            rclcpp::Time last_flag_time_;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            void on_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
            nav_msgs::msg::Odometry::SharedPtr odom_;
            VehicleState::SharedPtr state_;
            
        private:
            void generateCurrentState(State::SharedPtr state);

            std::ofstream csv_file_;
            rclcpp::Clock clock_; // 添加时钟成员变量
            bool initialized_ = false;
            void checkFlagTimeout();
    };
}
#endif
