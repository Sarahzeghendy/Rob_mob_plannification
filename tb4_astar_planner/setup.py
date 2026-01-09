from setuptools import setup

package_name = 'tb4_astar_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.py' , 'launch/astar_with_slam_rviz.launch.py',]),
        ('share/' + package_name + '/launch', ['launch/planner.launch.py' , 'launch/astar_with_saved_map_launch.py',]),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarah',
    maintainer_email='sarah@example.com',
    description='A* planner + obstacle inflation for TurtleBot4, publishes Path and follows it.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner_node = tb4_astar_planner.planner_node:main',
            'goal_input = tb4_astar_planner.goal_input_node:main', 
            'planner_node_with_saved_map = tb4_astar_planner.planner_node_with_saved_map:main',
            'static_map_publisher = tb4_astar_planner.static_map_publisher:main',
        ],
    },
)
