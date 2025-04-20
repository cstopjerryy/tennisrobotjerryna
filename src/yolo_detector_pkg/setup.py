from setuptools import setup

package_name = 'yolo_detector_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/' + package_name, ['package.xml', 'yolov8n.pt'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jerry',
    maintainer_email='cstopjerryy@gmail.com',
    description='YOLOv8 detector node',
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_detector_pkg.detector_node:main'
        ],
    },
)
