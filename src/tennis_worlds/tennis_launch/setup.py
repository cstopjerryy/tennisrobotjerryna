from setuptools import setup
import os
from glob import glob

package_name = 'tennis_launch'          # 包名字

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],            # 这里对应 tennis_launch/__init__.py
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # worlds  & launch 目录都 **相对 python 包** 来写
        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join(package_name, 'worlds', '*.world'))),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join(package_name, 'launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jerry Yang',
    maintainer_email='jerryyang@example.com',
    description='Tennis launch package',
    license='Apache 2.0',
    entry_points={'console_scripts': []},
)
