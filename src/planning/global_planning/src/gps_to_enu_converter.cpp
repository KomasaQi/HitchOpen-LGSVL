#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include <iostream>

class GPStoENUConverter : public rclcpp::Node
{
public:
    GPStoENUConverter() : Node("gps_to_enu_converter")
    {
        // 订阅 GPS 数据
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps_top/fix", 10,
            std::bind(&GPStoENUConverter::gpsCallback, this, std::placeholders::_1));

        initialized_ = false;
    }

private:
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (!initialized_)
        {
            // 初始化原点
            origin_lat_ = msg->latitude;
            origin_lon_ = msg->longitude;
            origin_alt_ = msg->altitude;

            const GeographicLib::Geocentric &earth = GeographicLib::Geocentric::WGS84();
            proj_ = GeographicLib::LocalCartesian(origin_lat_, origin_lon_, origin_alt_, earth);

            RCLCPP_INFO(this->get_logger(), "Origin GPS coordinates set: lat=%f, lon=%f, alt=%f",
                        origin_lat_, origin_lon_, origin_alt_);
            initialized_ = true;
        }
        else
        {
            // 转换 GPS 坐标为 ENU 坐标
            double east, north, up;
            proj_.Forward(msg->latitude, msg->longitude, msg->altitude, east, north, up);

            RCLCPP_INFO(this->get_logger(), "ENU coordinates: x=%f, y=%f, z=%f", east, north, up);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    GeographicLib::LocalCartesian proj_;
    double origin_lat_, origin_lon_, origin_alt_;
    bool initialized_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPStoENUConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}