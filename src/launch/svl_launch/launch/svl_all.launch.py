# Copyright 2022 AI Racing Tech
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from environs import Env

env = Env()
env.read_env("race.env")

svl_launch_dir = get_package_share_directory("svl_launch")
launch_dir = get_package_share_directory("autonomy_launch")
bs_launch_dir = get_package_share_directory("basestation_launch")
tools_launch_dir = get_package_share_directory("tools_launch")

svl_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(svl_launch_dir, "launch", "svl.launch.py")),
)

autonomy_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(launch_dir, "launch", "misc.launch.py"))
)

def generate_launch_description():
    return LaunchDescription([svl_launch, autonomy_launch])
