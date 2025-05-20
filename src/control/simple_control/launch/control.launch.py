from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    print("Generating launch description for control")
    params_file_path: Path = Path(get_package_share_directory('simple_control')) / 'params' / 'simple_control.yml'

    if not params_file_path.exists():
        raise FileNotFoundError(f"params file not found: {params_file_path}")

    ld = LaunchDescription()
    params_file = LaunchConfiguration('params_file')

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path.as_posix(),
        description='Path to the simple_control parameters file'
    ))
    ld.add_action(Node(
        package='simple_control',
        executable='controller_manager_node',
        name='controller_manager',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('/local_path', '/planning/local_path'),
            ('/control_cmd', '/control/control_cmd'),
            ('/odom', '/planning/odom')
        ]
    ))
    return ld