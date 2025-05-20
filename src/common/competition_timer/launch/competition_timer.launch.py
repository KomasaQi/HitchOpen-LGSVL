from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os
from environs import Env
import yaml

env = Env()
env.read_env("race.env")

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('competition_timer')
    
    # Load competition parameters
    competition_yaml = os.path.join(pkg_share, 'config', 'competition.yaml')
    with open(competition_yaml, 'r') as f:
        config = yaml.safe_load(f)
    
    map_name = env.str("MAP_NAME")
    map_config_path = os.path.join(pkg_share, 'config', f'{map_name}.yaml')

    # Competition timer node
    timer_node = Node(
        package='competition_timer',
        executable='competition_timer',
        name='competition_timer_node',
        output='screen',
        parameters=[{
            "results_dir": config['results_dir'],
            "use_sim_time": env.bool("USE_SIM_TIME"),
            "config_file_path": map_config_path,
            "target_laps": config['target_laps'],
            "vehicle_flag": config['vehicle_flag']
        }]
    )

    return LaunchDescription([

        timer_node
    ]) 