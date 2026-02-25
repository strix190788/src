from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Генерирует описание запуска для симуляции робота.
    """
    return LaunchDescription([
        # Узел батареи
        Node(
            package='my_robot_controller',
            executable='battery_node',
            name='battery_node',
            output='screen'  # вывод в консоль
        ),
        
        # Узел моторов
        Node(
            package='my_robot_controller',
            executable='motor_simulator',
            name='motor_simulator',
            output='screen'
        ),
        
        # Узел мониторинга
        Node(
            package='my_robot_controller',
            executable='system_monitor',
            name='system_monitor',
            output='screen'
        ),
    ])
