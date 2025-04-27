from setuptools import setup
import os
from glob import glob

package_name = 'tennis_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],                         # ⚡️这里直接留空
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.py')),            # 复制 launch/*.py
        ('share/' + package_name + '/worlds',
         glob('worlds/*.world')),          # 复制 worlds/*.world
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jerry Yang',
    maintainer_email='cstopjerryy@gmail.com',
    description='Tennis court simulation launch package',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [],
    },
)
