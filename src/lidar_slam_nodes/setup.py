from setuptools import find_packages, setup

package_name = 'lidar_slam_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhang Dong',
    maintainer_email='zhangdong1977@example.com',
    description='Python ROS2 nodes for LiDAR SLAM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = lidar_slam_nodes.frontier_explorer:main',
            'map_saver_watcher = lidar_slam_nodes.map_saver_watcher:main',
            'cmd_vel_bridge = lidar_slam_nodes.cmd_vel_bridge:main',
            'wait_for_tf = lidar_slam_nodes.wait_for_tf:main',
            'ackermann_keyboard_teleop = lidar_slam_nodes.ackermann_keyboard_teleop:main',
            'sim_teleop = lidar_slam_nodes.sim_teleop:main',
            'load_controllers = lidar_slam_nodes.load_controllers:main',
            'scan_range_filter = lidar_slam_nodes.scan_range_filter:main',

            'rs485_chassis_bridge = lidar_slam_nodes.rs485_chassis_bridge:main',
            'rs485_chassis_receiver = lidar_slam_nodes.rs485_chassis_receiver:main',
            'opentcs_vehicle_node = lidar_slam_nodes.opentcs_vehicle_node:main',
            'route_graph_loader = lidar_slam_nodes.route_graph_loader:main',
            'wait_for_topic = lidar_slam_nodes.wait_for_topic:main',
            'wait_for_service = lidar_slam_nodes.wait_for_service:main',
            'node_watchdog = lidar_slam_nodes.node_watchdog:main',
            'lifecycle_starter = lidar_slam_nodes.lifecycle_starter:main',
            'battery_bridge = lidar_slam_nodes.battery_bridge:main',
            'rviz_tf_bridge = lidar_slam_nodes.rviz_tf_bridge:main',
        ],
    },
)
