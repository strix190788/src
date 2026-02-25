from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Симуляторы датчиков (как бы на STM32)
        Node(
            package='my_robot_controller',
            executable='imu_simulator',
            name='imu_simulator',
            output='screen'
        ),
        Node(
            package='my_robot_controller',
            executable='encoder_simulator',
            name='encoder_simulator',
            output='screen'
        ),
        Node(
            package='my_robot_controller',
            executable='lidar_simulator',
            name='lidar_simulator',
            output='screen'
        ),
        
        # Обработка данных (на Raspberry Pi)
        Node(
            package='my_robot_controller',
            executable='imu_reader',
            name='imu_reader',
            output='screen'
        ),
        Node(
            package='my_robot_controller',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen'
        ),
    ])