from setuptools import setup

package_name = 'wavefront_frontier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
