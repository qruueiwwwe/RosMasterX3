from setuptools import setup
import os
from glob import glob

package_name = 'test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='测试移动控制包',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_test = test.move_test:main',
            'laser_processor = test.laser_processor:main',
            'depth_processor = test.depth_processor:main',
        ],
    },
) 