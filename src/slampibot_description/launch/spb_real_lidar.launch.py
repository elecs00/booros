#!/usr/bin/env python3
#
# Copyright 2023 EduRobotAILab CO., LTD.
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
#
# Author: Leo Cho

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
     
    rviz_config_file = os.path.join(get_package_share_directory('slampibot_description'), 'rviz', 'spb_urdf.rviz')        
    urdf_xacro = os.path.join(get_package_share_directory('slampibot_description'), 'urdf', 'spb_urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_topic = LaunchConfiguration('lidar_topic')
    robot_ip = LaunchConfiguration('robot_ip')
 
    dla_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    dla_lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/scan',
        description='LiDAR topic name from Raspberry Pi'
    )

    dla_robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.3.100',  # 라즈베리파이 IP 주소를 여기에 설정
        description='Raspberry Pi IP address'
    )
   
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': Command(['xacro',' ', urdf_xacro])}]
    )

    # TF static publisher for robot base
    static_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )

    # TF static publisher for LiDAR
    lidar_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'lidar_link'],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(dla_use_sim_time)
    ld.add_action(dla_lidar_topic)
    ld.add_action(dla_robot_ip)
    ld.add_action(rviz_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(static_tf_cmd)
    ld.add_action(lidar_tf_cmd)

    return ld 