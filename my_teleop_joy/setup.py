from setuptools import find_packages, setup

package_name = 'my_teleop_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_joy.py']),
        ('share/' + package_name + '/rviz', ['rviz/turtlebot_config.rviz']),
        ('share/' + package_name + '/launch', ['launch/rviz_launch.py']),
        ('share/' + package_name + '/launch', ['launch/inflate_map.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarah',
    maintainer_email='sarah@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_teleop_joy_node = my_teleop_joy.my_teleop_joy_node:main',
            'inflate_map_node = my_teleop_joy.inflate_map_node:main',
            'astar_planner_node = my_teleop_joy.astar_planner_node:main',
            'path_follower_node = my_teleop_joy.path_follower_node:main',
        ],
    },
)
