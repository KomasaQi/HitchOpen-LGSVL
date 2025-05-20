# Copyright 2022 AI Racing Tech
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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from base_common import get_share_file, get_sim_time_launch_arg, to_lower, check_val_in_list
from environs import Env

race_env = Env()
race_env.read_env("race.env")

geofence_inner_path = ""
geofence_outer_path = ""

if race_env.str("RACE_TYPE") == "IAC_PUTNAM" or race_env.str("RACE_TYPE") == "SVL_PUTNAM":
    geofence_inner_path = get_share_file(
        "race_metadata", "geo_fences", "PUTNAM_TIMES_A", "PUTNAM_INNER_GEOFENCE.csv"
    )
    geofence_outer_path = get_share_file(
        "race_metadata", "geo_fences", "PUTNAM_TIMES_A", "PUTNAM_OUTER_GEOFENCE.csv"
    )


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    rvc_config = (
        get_share_file("race_vehicle_controller"),
        "/param/",
        to_lower(LaunchConfiguration("vehicle_name")),
        "/rvc.param.yaml",
    )
    ttl_dir_arg = DeclareLaunchArgument(
        "ttl_dir", default_value="None", description="TTL Directory to use"
    )
    vehicle_model_config = (
        get_share_file("vehicle_model"),
        "/param/",
        to_lower(LaunchConfiguration("vehicle_name")),
        ".param.yaml",
    )
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            ttl_dir_arg,
            Node(
                package="race_vehicle_controller",
                executable="race_vehicle_controller_node_exe",
                name="race_vehicle_controller",
                output="screen",
                parameters=[
                    rvc_config,
                    vehicle_model_config,
                    use_sim_time,
                    {"ttl_directory": LaunchConfiguration("ttl_dir")},
                    {"fence_safety.inside_boundary_0": geofence_inner_path},
                    {"fence_safety.outside_boundary_0": geofence_outer_path},
                ],
                remappings=[
                    ("raw_command", "auto/raw_command"),
                    ("vehicle_kinematic_state", "/vehicle/state"),
                    ("trajectory_update", "/rde/trajectory_command"),
                    ("manual_command", "/rde/input_manual_command"),
                    ("engine_report", "/vehicle/engine_report"),
                    ("wheel_speeds", "/vehicle/wheel_speed_report"),
                    ("wheel_speed_report", "/vehicle/wheel_speed_report"),
                    ("push2pass_report", "/vehicle/push2pass_report"),
                ],
                emulate_tty=True,
                condition=IfCondition(
                    check_val_in_list(
                        "vehicle_name",
                        [
                            "IAC_CAR",
                            "SVL_IAC_CAR",
                            "AWSIM_IAC_CAR",
                            "HAWAII_GOKART",
                            "AWSIM_HAWAII_GOKART",
                        ],
                    )
                ),
            ),
        ]
    )
