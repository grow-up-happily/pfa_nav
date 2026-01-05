# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    waypoints_file = LaunchConfiguration("waypoints_file")
    
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace",
    )

    declare_waypoints_file_cmd = DeclareLaunchArgument(
        "waypoints_file",
        default_value="",
        description="Full path to waypoints.yaml file. If empty, will use CWD/waypoints.yaml",
    )

    hero_lidar_node = Node(
        package="hero_lidar",
        executable="hero_lidar",
        name="waypoint_static_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {"waypoints_file": waypoints_file},
        ],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_waypoints_file_cmd)

    # Add the node
    ld.add_action(hero_lidar_node)

    return ld
