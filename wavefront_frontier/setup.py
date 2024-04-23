from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wavefront_frontier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elvin Alberts',
    maintainer_email='',
    description='Wavefront Frontier Detector for ROS2 Navigation2',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'get_frontier_service = wavefront_frontier.frontier_service:main',
        ],
    },
)
