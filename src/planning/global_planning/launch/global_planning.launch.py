from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    print("Generating launch description for global_planning")
    ld = LaunchDescription()
    csv_file_path: Path = Path(get_package_share_directory('global_planning')) / 'params' / 'LVMS_SVL_ENU_TTL_15.csv'

    if not csv_file_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_file_path}")

    params_file_path: Path = Path(get_package_share_directory('global_planning')) / 'params' / 'global_planning.yml'
    if not params_file_path.exists():
        raise FileNotFoundError(f"params file not found: {params_file_path}")
    params_file = LaunchConfiguration('params_file')  

    ld.add_action(DeclareLaunchArgument(name="params_file", default_value=params_file_path.as_posix()))
    ld.add_action(Node(
        package='global_planning',
        executable='global_planning_node',
        name='global_planning',
        output='screen',
        parameters=[
            {'csv_file_path': csv_file_path.as_posix()},
            LaunchConfiguration('params_file'),
        ],
        remappings=[
            ('/gps_top/fix', '/gps_top/fix'),
            ('/planning/global_path', '/planning/global_path'),
            ('/planning/local_path', '/planning/local_path'),
            ('/raw/odom', '/odom'),
            ('/lgsvl/odometry', '/lgsvl/odom'),
            ('/planning/odom', '/planning/odom')
        ]
    ))
    return ld