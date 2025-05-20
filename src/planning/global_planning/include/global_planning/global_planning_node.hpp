#ifndef GLOBAL_PLANNING_NODE_HPP_
#define GLOBAL_PLANNING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <string>
#include <vector>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "lgsvl_msgs/msg/vehicle_odometry.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/msg/path.hpp>
struct Point3D {
    using SharedPtr = std::shared_ptr<Point3D>;
    double x;
    double y;
    double z;
};

class GlobalPlanningNode : public rclcpp::Node
{
public:
    explicit GlobalPlanningNode();
    ~GlobalPlanningNode() = default;

private:
    void loadWaypointsFromCSV(const std::string& csv_path);
    void publishGlobalPath();
    void publishLocalPath();
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    size_t findClosestWaypoint(
        const autoware_planning_msgs::msg::Path::SharedPtr path,
        const Point3D::SharedPtr current_position,
        const size_t search_radius,
        size_t prev_closest_index);
    size_t findNextWaypoint(
        const Point3D::SharedPtr current_position,
        const size_t search_radius);
    double calculateDistance(const Point3D& p1, const Point3D& p2) const;

    void p_convertGPStoENU(double latitude, double longitude, double altitude, Point3D* output) const;

    rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    
    autoware_planning_msgs::msg::Path::SharedPtr global_path_;
    
    std::string csv_file_path_;
    rclcpp::TimerBase::SharedPtr global_path_timer_;
    rclcpp::TimerBase::SharedPtr local_path_timer_;
    rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr local_path_pub_;
    double local_path_length_m_;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double origin_z_ = 0.0;

    GeographicLib::LocalCartesian proj;

    uint64_t current_waypoint_index_ = 0;
    bool received_gps_ = false;

    Point3D::SharedPtr current_position_;

    // subscribe to lgsvl_msgs/Odometry
    rclcpp::Subscription<lgsvl_msgs::msg::VehicleOdometry>::SharedPtr lgsvl_odom_sub_;
    void on_lgsvl_odom_callback(const lgsvl_msgs::msg::VehicleOdometry::SharedPtr msg);
    lgsvl_msgs::msg::VehicleOdometry::SharedPtr lgsvl_odom_;


    // subscribe to /odom
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void on_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    nav_msgs::msg::Odometry::SharedPtr odom_;

    // publish to planning/odom
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr planning_odom_pub_;
    rclcpp::TimerBase::SharedPtr planning_odom_timer_;
    void publishPlanningOdom();

    // visualizations using nav_msgs/Path
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_vis_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_vis_pub_;
    bool visualize_ = false;

    void convertAutowarePathToNavPath(const autoware_planning_msgs::msg::Path& autoware_path, nav_msgs::msg::Path& nav_path);
};

#endif  // GLOBAL_PLANNING_NODE_HPP_
