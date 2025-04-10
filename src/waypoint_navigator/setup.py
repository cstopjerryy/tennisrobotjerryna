from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # 指定Python子目录名字
    data_files=[
        # 让colcon正确识别本包
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 安装package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jerry',
    maintainer_email='test@example.com',
    description='A minimal waypoint navigation for TB3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 把节点入口点暴露出去
            'waypoint_navigator_node = waypoint_navigator.waypoint_navigator:main',
        ],
    },
)
