#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# The launch_descriptions describes a list of actions the `ros2 launch` command shall execute
def generate_launch_description():
    # --- Create links to additional, external launch files ---
        # --- Lidar Launch-Datei ---
    lidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_c1_launch.py'
    )

        # --- IMU Launch-Datei ---
    six_dof_imu_launch_file = os.path.join(
        get_package_share_directory('mpu6050driver'),
        'launch',
        'mpu6050driver_launch.py'
    )

        # --- RViz Config (optional anpassen) ---
    rviz_config = os.path.join(
        get_package_share_directory('imu_lidar_sensor_fusion'),
        'rviz2',
        'fusion_config.rviz'
    )

        # --- slam_toolbox Config ---
    slam_config = os.path.join(
        get_package_share_directory('imu_lidar_sensor_fusion'),
        'config',
        'mapper_params_online.yaml'
    )

    # --- Declare additional startup arguments ---
        # --- Launch-Argument für RViz---
    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',  # Is only used, if no value was provided in the `ros2 launch` cli command
        description='Starte RViz wenn true'
    )

    return LaunchDescription([
        # Start a lidar Node (via Launch-Datei eingebunden)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file)
        ),

        # Start an IMU Node (via Launch-Datei eingebunden)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(six_dof_imu_launch_file)
        ),

        # Start my custom fusion Node (my Python-Skript)
        Node(
            package='imu_lidar_sensor_fusion',
            executable='fusion_calculation_node',
            name='fusion_node',
            output='screen',
            parameters=[{'voxel_size': 0.05}]   # Kantenlänge eines Voxels in m
        ),

        # Start the SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # oder 'async_slam_toolbox_node'
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config]
        ),

        # Make the variable available in the launch context
        declare_rviz_arg,

        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ])
