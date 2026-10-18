from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'parking_lot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wt',
    maintainer_email='wt@todo.todo',
    description='Gazebo launch package for the parking-lot simulation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collector = parking_lot_sim.nodes.data_collector:main',
            'bev_projector = parking_lot_sim.nodes.bev_projector:main',
            'keyboard_teleop = parking_lot_sim.nodes.keyboard_teleop:main',
            'odom_tf_broadcaster = parking_lot_sim.nodes.odom_tf_broadcaster:main',
            'slam_diagnostics = parking_lot_sim.nodes.slam_diagnostics:main',
        ],
    },
)
