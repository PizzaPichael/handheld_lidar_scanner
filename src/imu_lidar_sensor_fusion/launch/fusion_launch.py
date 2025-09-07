#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Lidar Launch-Datei ---
    lidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_c1_launch.py'
    )

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
        # Lidar Node (via Launch-Datei eingebunden)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file)
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
