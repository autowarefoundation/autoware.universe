

"""
Setup for carla_manual_control
"""
import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(packages=['carla_autoware'], package_dir={'': 'src'})

    setup(**d)

elif ROS_VERSION == 2:
    from setuptools import setup

    package_name = 'carla_autoware'

    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ('share/' + package_name, glob('config/*.json')),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name), glob('launch/*.launch.xml'))
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='mradityagio',
        maintainer_email='mradityagio@gmail.com',
        description='CARLA ROS2 bridge for AUTOWARE',
        license='MIT',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': ['carla_autoware = carla_autoware.carla_autoware:main'],
        },
        package_dir={'': 'src'},
    )
