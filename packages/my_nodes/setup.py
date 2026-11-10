from setuptools import find_packages, setup

package_name = 'my_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='Мои ноды для курса Robomarvel',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello = my_nodes.hello:main',
            'talker = my_nodes.talker:main',
            'listener = my_nodes.listener:main',
            'battery_guard = my_nodes.battery_guard:main',
            'trip_meter = my_nodes.trip_meter:main',
            'gyro_yaw = my_nodes.gyro_yaw:main',
        ],
    },
)
