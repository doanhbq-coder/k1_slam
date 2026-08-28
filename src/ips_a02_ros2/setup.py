from glob import glob
import os
from setuptools import setup

package_name = 'ips_a02_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboticsvn',
    maintainer_email='roboticsvn.ai@gmail.com',
    description='Nooploop IPS A02 -> ROS2 (MQTT/UDP/Webhook) + hiển thị RViz2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ips_bridge = ips_a02_ros2.bridge_node:main',
            'ips_sim = ips_a02_ros2.sim_node:main',
            'ips_calib = ips_a02_ros2.calib_node:main',
            'ips_align = ips_a02_ros2.align_node:main',
            'ips_call_nav = ips_a02_ros2.call_nav_node:main',
            'ips_map_tf = ips_a02_ros2.map_tf_node:main',
        ],
    },
)
