#!/usr/bin/env python

# Copyright 2023 Husarion
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''Launch Webots rosbot_xl XL driver.'''

import os
import launch

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.actions import OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnProcessIO
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    PythonExpression,
    FindExecutable,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():

    params_file = LaunchConfiguration('params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('ros2_simulations'),
            'params',
            'webots_params_rosbot_xl.yaml',
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('ros2_simulations'),
            'rviz',
            'rviz_config_rosbot_xl.rviz',
        ]),
        description='Full path to the RVIZ config file to use'
    )

    slam = LaunchConfiguration('slam')
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether run a SLAM',
    )

    map_file = LaunchConfiguration('map_file')
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('ros2_simulations'),
            'maps',
            'map_husarion.yaml',
        ]),
        description='Full path to map yaml file to load',
    )

    #######################
    # ROSbot XL           #
    #######################

    rosbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('webots_ros2_husarion'),
                'launch',
                'rosbot_xl_launch.py',
            ])
        ),
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    tf_laser_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--z', '0.162', '--yaw', '3.1415927', '--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )

    #######################
    # Navigation          #
    #######################

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'True',
        }.items(),
    )

    #######################
    # Localization        #
    #######################

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'localization_launch.py',
            ])
        ),
        condition=IfCondition(PythonExpression(['not ', slam])),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
        }.items(),
    )

    #######################
    # Mapping             #
    #######################

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'slam_launch.py',
            ])
        ),
        condition=IfCondition(slam),
        launch_arguments={
            'params_file' : params_file,
            'use_sim_time': 'True',
        }.items(),
    )

    #######################
    # RViz                #
    #######################

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        declare_params_file_cmd,
        declare_use_rviz_cmd,
        declare_rviz_config_file_cmd,
        declare_slam_cmd,
        declare_map_file_cmd,
        rosbot_launch,
        rviz_node,
        footprint_publisher,
        navigation_launch,
        TimerAction(
            period=3.0,
            actions=[tf_laser_publisher]
        ),
        TimerAction(
            period=5.0,
            actions=[slam_launch]
        ),
        TimerAction(
            period=5.0,
            actions=[localization_launch]
        )
    ])
