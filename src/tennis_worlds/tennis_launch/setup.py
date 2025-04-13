from setuptools import setup
import os
from glob import glob

package_name = 'tennis_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],                # 让它成为可 import 的包
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # worlds
        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join(package_name, 'worlds', '*.world'))),
        # launch
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join(package_name, 'launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jerry Yang',
    maintainer_email='jerryyang@example.com',
    description='Tennis launch package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
