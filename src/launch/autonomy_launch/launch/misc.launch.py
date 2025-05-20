from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from base_common import get_share_file

from environs import Env

env = Env()
env.read_env("race.env")

if env.str("VEHICLE_NAME") == "IAC_CAR" or env.str("VEHICLE_NAME") == "SVL_IAC_CAR":
    vehicle_name_arg = DeclareLaunchArgument(
        name="urdf_path",
        default_value=get_share_file("race_metadata", "urdf", "iac_car", "av24.urdf"),
        description="Vehicle we are running",
    )
else:
    raise ValueError("Invalid vehicle name")

gui_arg = DeclareLaunchArgument(
    name="gui",
    default_value="false",
    choices=["true", "false"],
    description="Flag to enable joint_state_publisher_gui",
)

robot_description = ParameterValue(
    Command(["xacro ", LaunchConfiguration("urdf_path")]), value_type=str
)

declare_use_sim_time_cmd = DeclareLaunchArgument(
    name="use_sim_time", default_value=str(env.bool("USE_SIM_TIME"))
)

robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[
        {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_description": robot_description,
        }
    ],
)

# Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
joint_state_publisher_node = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    condition=UnlessCondition(LaunchConfiguration("gui")),
)

joint_state_publisher_gui_node = Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
    parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    condition=IfCondition(LaunchConfiguration("gui")),
)



def generate_launch_description():
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            vehicle_name_arg,
            gui_arg,
            robot_state_publisher_node,
        ]
    )
