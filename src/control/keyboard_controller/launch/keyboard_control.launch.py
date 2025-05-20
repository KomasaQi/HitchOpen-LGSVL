from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('keyboard_controller')
    default_output_dir = "./src/control/keyboard_controller/collected_waypoints"
    default_config_file = os.path.join(pkg_dir, 'config', 'LVMS.yaml')
    
    # Load the YAML file
    with open(default_config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Extract origin coordinates
    origin = config['origin']
    
    # Declare launch arguments
    save_waypoints_arg = DeclareLaunchArgument(
        'save_waypoints',
        default_value='true',
        description='Whether to save waypoints when quitting (true/false)'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=default_output_dir,
        description='Directory where waypoint files will be saved'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to YAML config file containing origin coordinates'
    )

    # Create the node
    keyboard_control_node = Node(
        package='keyboard_controller',
        executable='keyboard_control',
        name='keyboard_control_node',
        output='screen',
        parameters=[{
            'save_waypoints': LaunchConfiguration('save_waypoints'),
            'output_dir': LaunchConfiguration('output_dir'),
            'origin_x': origin['X'],
            'origin_y': origin['Y'],
            'origin_z': origin['Z']
        }]
    )

    # Return the launch description
    return LaunchDescription([
        save_waypoints_arg,
        output_dir_arg,
        config_file_arg,
        keyboard_control_node
    ]) 