from setuptools import setup
import os
from glob import glob

package_name = 'terrain_mapping_drone_control'

def package_tree(path):
    data_files = []
    for root, _, files in os.walk(path):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, root)
        data_files.append((install_dir, [os.path.join(root, name) for name in files]))
    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*')),
        ('share/' + package_name + '/worlds',
            glob('worlds/*.sdf')),
    ] + package_tree('models/px4_models') + package_tree('models/OakD-Lite-Modify') + package_tree('models/martian_surface'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='PX4/Gazebo Mars RVIO evaluation pipeline using OpenVINS range features',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rvio_evaluator_node = terrain_mapping_drone_control.rvio_evaluator_node:main',
            'rvio_rqt_plot_launcher = terrain_mapping_drone_control.rvio_rqt_plot_launcher:main',
            'rvio_rqt_plot_retry = terrain_mapping_drone_control.rvio_rqt_plot_retry:main',
            'px4_imu_bridge = terrain_mapping_drone_control.px4_imu_bridge_node:main',
            'openvins_on_state_launcher = terrain_mapping_drone_control.openvins_on_state_launcher:main',
            'pose_visualizer = terrain_mapping_drone_control.pose_visualizer:main',
            'rangefinder_terrain_mission = terrain_mapping_drone_control.rangefinder_terrain_mission:main',
        ],
    },
    python_requires='>=3.8'
) 
