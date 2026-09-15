from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.rviz') + glob('config/*.yaml')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyx6662',
    maintainer_email='lyx6662@todo.todo',
    description='无实物阶段的硬件模拟节点(底盘/GPS/雷达)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_base = robot_sim.mock_base:main',
            'mock_gps = robot_sim.mock_gps:main',
            'mock_lidar = robot_sim.mock_lidar:main',
            'mock_gimbal_camera = robot_sim.mock_gimbal_camera:main',
            'patrol_manager = robot_sim.patrol_manager:main',
            'map_manager = robot_sim.map_manager:main',
            'web_server = robot_sim.web_server:main',
        ],
    },
)
