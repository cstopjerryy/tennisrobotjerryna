from setuptools import setup, find_packages

package_name = 'ball_follower_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools', 'opencv-python', 'cv_bridge'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            # 名称 = 模块路径:main
            'ball_follower = ball_follower_pkg.ball_follower_node:main',
        ],
    },
)
