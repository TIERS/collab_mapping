import os
from glob import glob
from setuptools import setup

package_name = 'collab_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'models'), glob('resource/models/*.*')),
        (os.path.join('share', package_name, 'models/meshes'), glob('resource/models/meshes/*')),
        (os.path.join('share', package_name, 'waypoints'), glob('resource/waypoints/*')),
        (os.path.join('share', package_name, 'worlds'), glob('resource/worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('resource/rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('resource/config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farhad',
    maintainer_email='fakera@utu.fi',
    description='A collabarative mapping simulation in Gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_control = collab_mapping.turtlebot_control:main',
            'waypoint_recorder = collab_mapping.record_waypoints:main',
            'run_sc = collab_mapping.smartcontract:main',
            'laser_filter = collab_mapping.laser_filter:main'
        ],
    },
)
