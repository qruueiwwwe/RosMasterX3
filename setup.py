from setuptools import setup
import os
from glob import glob

package_name = 'rosmaster_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='ROS2 driver for ROSMASTER X3 robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosmaster_driver = rosmaster_driver.rosmaster_driver_ros2:main',
        ],
    },
) 