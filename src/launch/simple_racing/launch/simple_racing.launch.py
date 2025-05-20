from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    params_file_path: Path = Path(get_package_share_directory('simple_racing')) / 'params' / 'simple_racing.yml'
    if not params_file_path.exists():
        raise FileNotFoundError(f"params file not found: {params_file_path}")
    params_file = LaunchConfiguration('params_file')
    ld.add_action(DeclareLaunchArgument(name="params_file", default_value=params_file_path.as_posix()))
    
    # Get the launch file paths
    global_planning_launch = Path(get_package_share_directory('global_planning')) / 'launch' / 'global_planning.launch.py'
    control_launch = Path(get_package_share_directory('simple_control')) / 'launch' / 'control.launch.py'
    interface_launch = Path(get_package_share_directory('autoware_lgsvl_interface')) / 'launch' / 'autoware_lgsvl_interface.launch.py'
    # Verify launch files exist
    if not global_planning_launch.exists():
        raise FileNotFoundError(f"Launch file not found: {global_planning_launch}")
    if not control_launch.exists():
        raise FileNotFoundError(f"Launch file not found: {control_launch}")
    if not interface_launch.exists():
        raise FileNotFoundError(f"Launch file not found: {interface_launch}")
    # Include both launch files
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(global_planning_launch.as_posix()),
        launch_arguments={'params_file': params_file}.items()
    ))
    
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch.as_posix()),
        launch_arguments={'params_file': params_file}.items()
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(interface_launch.as_posix()),
        launch_arguments={'params_file': params_file}.items()
    ))

    return ld
