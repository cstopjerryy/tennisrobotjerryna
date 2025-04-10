from setuptools import setup
import os
from glob import glob

package_name = 'tennis_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ✅ 注意路径写 tennis_launch/*.world 是因为 .world 在这个包目录下
        (os.path.join('share', package_name, 'worlds'), glob('tennis_launch/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('tennis_launch/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jerry Yang',
    maintainer_email='jerryyang@example.com',
    description='Tennis launch package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
