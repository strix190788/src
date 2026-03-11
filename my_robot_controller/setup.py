from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Добавляем launch файлы
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # URDF файлы
        (os.path.join('share', package_name, 'urdf'), 
            glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'my_first_node = my_robot_controller.my_first_node:main',
        'robot_news_station = my_robot_controller.robot_news_station:main',
        'smartphone = my_robot_controller.smartphone:main',
        'battery_node = my_robot_controller.battery_node:main',
        'motor_simulator = my_robot_controller.motor_simulator:main',
        'system_monitor = my_robot_controller.system_monitor:main',
        'imu_simulator = my_robot_controller.imu_simulator:main',
        'imu_reader = my_robot_controller.imu_reader:main',
        'encoder_simulator = my_robot_controller.encoder_simulator:main',
        'wheel_odometry = my_robot_controller.wheel_odometry:main',
        'lidar_simulator = my_robot_controller.lidar_simulator:main',
        'static_transform_publisher = my_robot_controller.static_transform_publisher:main',
        'odometry_error_demo = my_robot_controller.odometry_error_demo:main',
        'calibration_helper = my_robot_controller.calibration_helper:main',
        'slip_detector = my_robot_controller.slip_detector:main',
        'sensor_fusion = my_robot_controller.sensor_fusion:main',
        'robot_tf_broadcaster = my_robot_controller.robot_tf_broadcaster:main',
        'lidar_room_simulator = my_robot_controller.lidar_room_simulator:main',
        'lidar_processor = my_robot_controller.lidar_processor:main',
        ],
    },
)