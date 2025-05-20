# Copyright 2024 AI Racing Tech
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution
from launch.actions import DeclareLaunchArgument
from base_common import get_param_file, get_sim_time_launch_arg


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    origin_lat_arg = DeclareLaunchArgument(
        "origin_lat", default_value="0.0", description="Latitude of the map origin"
    )
    origin_lon_arg = DeclareLaunchArgument(
        "origin_lon", default_value="0.0", description="Longitude of the map origin"
    )
    origin_alt_arg = DeclareLaunchArgument(
        "origin_alt", default_value="0.0", description="Altitude of the map origin"
    )
    vehicle_name_arg = DeclareLaunchArgument("vehicle_name")
    vks_param_file = get_param_file("vehicle_kinematic_state", launch_config_var="vehicle_name")

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            origin_lat_arg,
            origin_lon_arg,
            origin_alt_arg,
            vehicle_name_arg,
            Node(
                package="vehicle_kinematic_state",
                executable="vehicle_kinematic_state_node",
                name="vehicle_kinematic_state_node",
                output="screen",
                parameters=[
                    vks_param_file,
                    use_sim_time,
                    {
                        "map_origin.latitude": LaunchConfiguration("origin_lat"),
                        "map_origin.longitude": LaunchConfiguration("origin_lon"),
                        "map_origin.altitude": LaunchConfiguration("origin_alt"),
                    },
                ],
                remappings=[
                    ("raw_gps_1", "/gps_bot/gps"),
                    ("raw_imu_1", "/gps_bot/imu"),
                    ("raw_gps_2", "/gps_top/gps"),
                    ("raw_imu_2", "/gps_top/imu"),
                    ("steering_report", "/vehicle/steering_report"),
                    ("vehicle_kinematic_state", "/vehicle/state"),
                    ("heading_diff", "/car_heading_stamped"),
                    ("gps_odom", "/odometry/gps"),
                    ("filtered_odom", "/odometry/filtered"),
                    ("acceleration", "/accel/filtered"),
                    ("imu", "/odometry/imu/filtered"),
                ],
                emulate_tty=True,
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration("vehicle_name"), "IAC_CAR")
                ),
            ),
            Node(
                package="vehicle_kinematic_state",
                executable="vehicle_kinematic_state_node",
                name="vehicle_kinematic_state_node",
                output="screen",
                parameters=[
                    vks_param_file,
                    use_sim_time,
                    {
                        "map_origin.latitude": LaunchConfiguration("origin_lat"),
                        "map_origin.longitude": LaunchConfiguration("origin_lon"),
                        "map_origin.altitude": LaunchConfiguration("origin_alt"),
                    },
                ],
                remappings=[
                    ("raw_gps_1", "/gps_bot/gps"),
                    ("raw_imu_1", "/gps_bot/imu"),
                    # ("raw_gps_2", "/gps_front/gps"),
                    # ("raw_imu_2", "/gps_front/imu"),
                    ("steering_report", "/vehicle/steering_report"),
                    ("vehicle_kinematic_state", "/vehicle/state"),
                    ("heading_diff", "/car_heading_stamped"),
                    ("gps_odom", "/odometry/gps"),
                    ("filtered_odom", "/odometry/filtered"),
                    ("acceleration", "/accel/filtered"),
                    ("imu", "/odometry/imu/filtered"),
                ],
                emulate_tty=True,
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration("vehicle_name"), "SVL_IAC_CAR")
                ),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="global_extended_kalman_filter_node",
                output="screen",
                remappings=[
                    ("odom0", "/odometry/gps"),
                    ("odom1", "/vehicle/odometry"),
                    ("imu0", "/gps_bot/imu"),
                    ("imu1", "/gps_top/imu"),
                ],
                parameters=[
                    vks_param_file,
                    use_sim_time,
                ],
                emulate_tty=True,
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration("vehicle_name"), "IAC_CAR")
                ),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="global_extended_kalman_filter_node",
                output="screen",
                remappings=[
                    ("odom0", "/odometry/gps"),
                    ("imu0", "/gps_bot/imu"),
                ],
                parameters=[
                    vks_param_file,
                    use_sim_time,
                ],
                emulate_tty=True,
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration("vehicle_name"), "SVL_IAC_CAR")
                ),
            ),
        ]
    )
