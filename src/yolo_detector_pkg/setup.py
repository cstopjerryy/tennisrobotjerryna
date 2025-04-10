from setuptools import find_packages, setup

package_name = 'yolo_detector_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jerryyang',
    maintainer_email='jerryyang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'yolo_detector_node = yolo_detector_pkg.yolo_detector_node:main',
        'test_image_publisher = yolo_detector_pkg.test_image_publisher:main',
        ],
    },
)
