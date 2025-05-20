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
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from base_common import get_sim_time_launch_arg, get_param_file


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    vehicle_name_arg = DeclareLaunchArgument("vehicle_name")
    vos_param_file = get_param_file("odometry_state_estimator", launch_config_var="vehicle_name")
    ttl_dir_arg = DeclareLaunchArgument(
        "ttl_dir", default_value="None", description="TTL Directory to use"
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            vehicle_name_arg,
            ttl_dir_arg,
            Node(
                package="odometry_state_estimator",
                executable="odometry_state_estimator_node",
                name="odometry_state_estimator_node",
                output="screen",
                parameters=[
                    vos_param_file,
                    use_sim_time,
                    {"ttl_dir": LaunchConfiguration("ttl_dir")},
                ],
                remappings=[
                    ("wheel_speed_report", "/vehicle/wheel_speed_report"),
                    ("odom", "/vehicle/odometry"),
                    ("imu", "/odometry/imu/filtered"),
                    ("odom_correction", "/odometry/gps"),
                    ("target_trajectory_command", "/rde/trajectory_command"),
                    ("steering_report", "/vehicle/steering_report"),
                ],
                emulate_tty=True,
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration("vehicle_name"), "IAC_CAR")
                ),
                prefix=["nice -n -5"],
            ),
        ]
    )
