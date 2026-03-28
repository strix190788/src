import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Пути к файлам
    pkg_share = get_package_share_directory('my_nav2_config')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    map_yaml_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_nav2.rviz')
    
    # Аргументы
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Запуск Map Server
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_file}
        ]
    )
    
    # Запуск AMCL (локализация)
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Запуск Nav2 (все компоненты)
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file
        }.items()
    )
    
    # Запуск RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Lifecycle manager для Map Server и AMCL
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']}
        ]
    )
    
    ld = LaunchDescription()
    
    # Добавляем действия
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_cmd)
    
    return ld
