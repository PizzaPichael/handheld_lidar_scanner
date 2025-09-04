#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Lidar Parameter ---
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    # --- IMU Launch-Datei ---
    imu_launch_file = os.path.join(
        get_package_share_directory('mpu6050driver'),
        'launch',
        'mpu6050driver_launch.py'
    )

    # --- RViz Config (optional anpassen) ---
    rviz_config = os.path.join(
        get_package_share_directory('imu_lidar_sensor_fusion'),
        'rviz',
        'fusion_config.rviz'
    )

    # --- slam_toolbox Config ---
    slam_config = os.path.join(
        get_package_share_directory('imu_lidar_sensor_fusion'),
        'config',
        'mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        # Lidar Argumente
        DeclareLaunchArgument('channel_type', default_value=channel_type,
                              description='Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port', default_value=serial_port,
                              description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate,
                              description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument('frame_id', default_value=frame_id,
                              description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted', default_value=inverted,
                              description='Invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate,
                              description='Enable angle compensation'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode,
                              description='Lidar scan mode'),

        # Lidar Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode
            }],
            output='screen'
        ),

        # IMU Node (via Launch-Datei eingebunden)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch_file)
        ),

        # Fusion Node (mein Python-Skript)
        Node(
            package='imu_lidar_sensor_fusion',
            executable='fusion_calculation_node',
            name='fusion_node',
            output='screen'
        ),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # oder 'async_slam_toolbox_node'
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
