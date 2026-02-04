from setuptools import setup
import os
from glob import glob

package_name = 'tb4_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_name + '.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'pyyaml',
        'pillow',
    ],
    zip_safe=True,
    maintainer='Sarah',
    maintainer_email='your_email@example.com',
    description='Modular navigation system for TurtleBot4 with A* planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_manager = tb4_navigation.map_manager:main',
            'path_follower = tb4_navigation.path_follower:main',
            'navigation_controller = tb4_navigation.navigation_controller:main',
            'exploration_manager = tb4_navigation.exploration_manager:main',
            'goal_input = tb4_navigation.goal_input:main',
            'trip_handler = tb4_navigation.trip_handler:main',
        ],
    },
)
