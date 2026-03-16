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
    pose_topic = LaunchConfiguration("pose_topic")
    publish_rate = LaunchConfiguration("publish_rate")
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud_rate = LaunchConfiguration("serial_baud_rate")
    
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

    declare_pose_topic_cmd = DeclareLaunchArgument(
        "pose_topic",
        default_value="/hero_pose",
        description="PoseStamped topic used to update map->base.",
    )

    declare_publish_rate_cmd = DeclareLaunchArgument(
        "publish_rate",
        default_value="10.0",
        description="Continuous TF publish rate in Hz.",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation clock. Set to False on real hardware.",
    )

    declare_serial_port_cmd = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/gimbal",
        description="Serial device used for sending gimbal packets.",
    )

    declare_serial_baud_rate_cmd = DeclareLaunchArgument(
        "serial_baud_rate",
        default_value="115200",
        description="Baud rate used for the gimbal serial device.",
    )

    hero_lidar_node = Node(
        package="hero_lidar",
        executable="hero_lidar",
        name="waypoint_static_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {"waypoints_file": waypoints_file},
            {"pose_topic": pose_topic},
            {"publish_rate": publish_rate},
            {"use_sim_time": use_sim_time},
            {"serial_port": serial_port},
            {"serial_baud_rate": serial_baud_rate},
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_waypoints_file_cmd)
    ld.add_action(declare_pose_topic_cmd)
    ld.add_action(declare_publish_rate_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_serial_port_cmd)
    ld.add_action(declare_serial_baud_rate_cmd)

    # Add the node
    ld.add_action(hero_lidar_node)

    return ld
