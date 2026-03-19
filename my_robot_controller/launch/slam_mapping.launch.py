#!/usr/bin/env python3
"""
Launch-файл для построения карты с помощью SLAM Toolbox.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Путь к конфигурации
    pkg_dir = get_package_share_directory('my_robot_controller')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    
    return LaunchDescription([
        # ═══════════════════════════════════════════════════
        # СИМУЛЯТОР МИРА
        # ═══════════════════════════════════════════════════
        Node(
            package='my_robot_controller',
            executable='slam_world_simulator',
            name='slam_world_simulator',
            output='screen',
        ),
        
        # ═══════════════════════════════════════════════════
        # SLAM TOOLBOX
        # ═══════════════════════════════════════════════════
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
        ),
        
        # ═══════════════════════════════════════════════════
        # ВИЗУАЛИЗАЦИЯ В RVIZ
        # ═══════════════════════════════════════════════════
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'slam.rviz')],
        ),
        
        # ═══════════════════════════════════════════════════
        # FOXGLOVE BRIDGE (опционально)
        # ═══════════════════════════════════════════════════
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ),
    ])
