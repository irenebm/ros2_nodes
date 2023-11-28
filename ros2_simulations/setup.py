import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'ros2_simulations'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/gazebo_rosbot_xl.launch.py']))
data_files.append(('share/' + package_name + '/urdf', [
        'urdf/rosbot_xl_macro.urdf.xacro'
]))
data_files.append(('share/' + package_name + '/params', [
        'params/nav2_params_rosbot_xl.yaml',
        'params/slam_params_rosbot_xl.yaml',
        'params/navigate_w_recovery.xml'
]))
data_files.append(('share/' + package_name + '/rviz', [
        'rviz/rviz_config_rosbot_xl.rviz'
]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo',
    description='todo',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    }
)
