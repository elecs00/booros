#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    slampibot_description_dir = get_package_share_directory('slampibot_description')
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    
    # URDF file path
    urdf_file = os.path.join(slampibot_description_dir, 'urdf', 'spb_urdf_real_a1.urdf')
    
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for A1 lidar'
        ),
        
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='Serial baudrate for A1 lidar'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Frame ID for lidar data'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_tf_static': True}],
            arguments=[urdf_file]
        ),
        
        # A1 Lidar node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
            }],
            output='screen'
        ),
        
        # RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(slampibot_description_dir, 'rviz', 'spb_urdf.rviz')],
            output='screen'
        ),
        
        # Joint state publisher (for visualization)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    ]) 