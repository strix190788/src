#!/usr/bin/env python3
"""
Launch-файл для сохранения построенной карты.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        # Аргумент для имени файла карты
        DeclareLaunchArgument(
            'map_name',
            default_value='my_apartment',
            description='Name of the map to save'
        ),
        
        # Сохранение карты через map_saver_cli
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', ['/home/student/maps/', LaunchConfiguration('map_name')]
            ],
            output='screen'
        ),
        
        # Альтернативный способ через сервис slam_toolbox
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/slam_toolbox/save_map',
                'slam_toolbox/srv/SaveMap',
                ['{name: {data: "/home/student/maps/', LaunchConfiguration('map_name'), '"}}']
            ],
            output='screen'
        ),
    ])