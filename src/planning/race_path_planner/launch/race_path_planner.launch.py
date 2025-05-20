from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from base_common import get_param_file, get_sim_time_launch_arg, check_val_in_list


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    race_type_arg = DeclareLaunchArgument(
        "race_type", default_value="None", description="Race Type"
    )
    ttl_dir_arg = DeclareLaunchArgument(
        "ttl_dir", default_value="None", description="TTL Directory to use"
    )
    config = get_param_file("race_path_planner")

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            ttl_dir_arg,
            race_type_arg,
            Node(
                package="race_path_planner",
                executable="race_path_planner_node_exe",
                parameters=[
                    config,
                    use_sim_time,
                    {"ttl_directory": LaunchConfiguration("ttl_dir")},
                ],
                remappings=[
                    ("local_trajectory", "/rpp/path_command"),
                    ("trajectory_viz", "/rpp/trajectory_viz"),
                    ("rpp_command", "/rpp/trajectory_command"),
                    ("rpp_telemetry", "/rpp/telemetry"),
                    ("state", "/vehicle/state"),
                    ("detected_objects", "/tracked_objects"),
                    ("trajectory_command", "/rde/trajectory_command"),
                ],
                emulate_tty=True,
                condition=IfCondition(
                    check_val_in_list(
                        "race_type",
                        [
                            "IAC_LVMS",
                            "SVL_LVMS",
                            "AW_SIM_IAC_LVMS",
                            "HAWAII_GOKART_AAIS",
                            "AW_SIM_HAWAII_GOKART_AAIS",
                        ],
                    )
                ),
            ),
        ]
    )
