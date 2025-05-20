// Copyright 2022 AI Racing Tech
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <memory>
#include <ctime>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <limits>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include "race_vehicle_controller/plugin.hpp"
#include "rclcpp/rclcpp.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;

#define SECONDS_TO_MILLISECONDS 1000

namespace race
{

class CheckOutsideTrack : public RvcPlugin
{
public:
  bool curr_timeout_state = false;
  std::chrono::time_point<std::chrono::system_clock> last_inside_time;
  double timeout_duration;
  std::chrono::time_point<std::chrono::system_clock> manual_disable_time_;
  //  Duration for which geofence is manually disabled (in seconds)
  double manual_disable_duration_ = 0;
  bool manual_disable_geofence_;
  double geofence_trigger_distance_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

  std::vector<std::pair<std::vector<Point>, std::vector<Point>>> boundaries;
  std::vector<Point> readCSV(const std::string & filename)
  {
    std::vector<Point> data;

    std::ifstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Error opening file: " << filename << std::endl;
      return data;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::vector<double> row;
      std::stringstream ss(line);
      std::string cell;
      int i = 0;
      while (std::getline(ss, cell, ',')) {
        i++;
        try {
          double value = std::stod(cell);
          row.push_back(value);
        } catch (const std::exception & e) {
          std::cerr << "Error converting cell to double: " << cell << std::endl;
        }
      }
      data.push_back(Point(row[0], row[1]));
    }

    file.close();
    return data;
  }

  void readBoundaries(const std::string & prefix)
  {
    int i = 0;
    while (true) {
      std::string inside_param = prefix + ".inside_boundary_" + std::to_string(i);
      std::string outside_param = prefix + ".outside_boundary_" + std::to_string(i);
      rclcpp::Parameter parameter;
      node().declare_parameter(inside_param, rclcpp::PARAMETER_STRING);
      node().declare_parameter(outside_param, rclcpp::PARAMETER_STRING);
      if (!node().get_parameter(
          inside_param,
          parameter) || !node().get_parameter(outside_param, parameter)) {break;}

      std::string inside_boundary = node().get_parameter(inside_param).as_string();
      std::string outside_boundary = node().get_parameter(outside_param).as_string();

      boundaries.push_back({readCSV(inside_boundary), readCSV(outside_boundary)});
      i++;
    }
  }

  bool inside_region(Point pt, std::vector<Point> & pgn, K traits, double allowed_distance = 0.0)
  {
    switch (CGAL::bounded_side_2(pgn.begin(), pgn.end(), pt, traits)) {
      case CGAL::ON_BOUNDED_SIDE:
        return true;
      case CGAL::ON_BOUNDARY:
        return true;
      case CGAL::ON_UNBOUNDED_SIDE:
        if (allowed_distance > 0.0) {
          double dist = distance_from_point_to_polygon(pt, pgn);
          if (dist <= allowed_distance) {
            return true;
          }
        }
        return false;
    }
    return true;
  }


  double distance_from_point_to_polygon(Point pt, const std::vector<Point> & pgn)
  {
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < pgn.size(); ++i) {
      const Point & p1 = pgn[i];
      const Point & p2 = pgn[(i + 1) % pgn.size()];
      double distance = std::sqrt(CGAL::squared_distance(pt, CGAL::Segment_2<K>(p1, p2)));

      if (distance < min_distance) {
        min_distance = distance;
      }
    }
    return min_distance;
  }

  bool check_inside(double x, double y)
  {
    Point pt(x, y);
    for (auto & boundary : boundaries) {
      if (inside_region(pt, boundary.second, K(), geofence_trigger_distance_) &&
        !inside_region(pt, boundary.first, K(), geofence_trigger_distance_))
      {
        return true;
      }
    }
    return false;
  }

  bool configure() override
  {
    last_inside_time = std::chrono::system_clock::now();
    timeout_duration = 1.0;
    publisher_ = node().create_publisher<std_msgs::msg::Bool>(
      "geofence_telemetry",
      rclcpp::SensorDataQoS());


    readBoundaries("fence_safety");

    node().declare_parameter<bool>("manual_disable_geofence", false);
    node().get_parameter("manual_disable_geofence", manual_disable_geofence_);

    geofence_trigger_distance_ = node().declare_parameter<double>("fence_safety.trigger_distance");

    return true;
  }

  bool is_inside_boundary()
  {
    double curr_x = const_state().kin_state->pose.pose.position.x;
    double curr_y = const_state().kin_state->pose.pose.position.y;
    return check_inside(curr_x, curr_y);
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    const auto & name = param.get_name();
    const auto & type = param.get_type();
    if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == "manual_disable_geofence") {
        manual_disable_geofence_ = param.as_bool();
        return true;
      }
    }
    return false;
  }

  bool modify_control_command(VehicleControlCommand & output_cmd) override
  {
    auto curr_time = std::chrono::system_clock::now();
    if (manual_disable_geofence_) {
      // If the geofence is manually disabled, skip the checks
      return true;
    }

    double elapsed_manual_disable_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      curr_time - manual_disable_time_).count() / SECONDS_TO_MILLISECONDS;

    if (manual_disable_duration_ > 0 && elapsed_manual_disable_time <= manual_disable_duration_) {
      return true;
    }

    auto msg = std_msgs::msg::Bool();
    if (!is_inside_boundary()) {
      curr_time = std::chrono::system_clock::now();
      msg.data = true;

      if (std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - last_inside_time).count() / SECONDS_TO_MILLISECONDS > timeout_duration)
      {
        curr_timeout_state = true;
        output_cmd.steering_cmd = 0.;
        output_cmd.accelerator_cmd = 0.;
        output_cmd.brake_cmd = model().get_config().rear_brake_config->max_brake;
      }
    } else {
      msg.data = false;
      last_inside_time = std::chrono::system_clock::now();
    }

    publisher_->publish(msg);
    return true;
  }

  const char * get_plugin_name() override
  {
    return "Check Outside Track";
  }
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::CheckOutsideTrack, race::RvcPlugin)
