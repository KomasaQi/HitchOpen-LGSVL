#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include <iostream>
#include <fstream>

class GPStoENUConverter : public rclcpp::Node
{
public:
    GPStoENUConverter() : Node("enu_logger")
    {
        // 订阅 GPS 数据
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps_top/fix", 10,
            std::bind(&GPStoENUConverter::gpsCallback, this, std::placeholders::_1));

        initialized_ = false;
        // 打开 CSV 文件
        csv_file_.open("output1.csv");
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file.");
        }
    }

    ~GPStoENUConverter()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
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
            // 写入 CSV 文件的第一行
            if (csv_file_.is_open()) {
                csv_file_ << "0,0,0," << origin_lat_ << "," << origin_lon_ << "," << origin_alt_ << std::endl;
            }
            initialized_ = true;
        }
        else
        {
            // 转换 GPS 坐标为 ENU 坐标
            double east, north, up;
            proj_.Forward(msg->latitude, msg->longitude, msg->altitude, east, north, up);

            RCLCPP_INFO(this->get_logger(), "ENU coordinates: x=%f, y=%f, z=%f", east, north, up);
            // 写入后续行到 CSV 文件
            if (csv_file_.is_open()) {
                csv_file_ << east << "," << north << "," << up << ",0,25" << std::endl;
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    GeographicLib::LocalCartesian proj_;
    double origin_lat_, origin_lon_, origin_alt_;
    bool initialized_;
    
    std::ofstream csv_file_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPStoENUConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}    