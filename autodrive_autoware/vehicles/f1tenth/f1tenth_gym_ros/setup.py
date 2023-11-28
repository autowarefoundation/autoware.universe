from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_gym_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chinmay Vilas Samak' 'Tanmay Vilas Samak' 'Billy Zheng',
    maintainer_email='csamak@clemson.edu' 'tsamak@clemson.edu' 'billyzheng.bz@gmail.com',
    description='Bridge for using f1tenth_gym in ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gym_bridge = f1tenth_gym_ros.gym_bridge:main'
        ],
    },
)