from setuptools import setup
import os
from glob import glob

package_name = 'particle_filter'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chinmay Vilas Samak' 'Tanmay Vilas Samak' 'Hongrui Zheng',
    maintainer_email='csamak@clemson.edu' 'tsamak@clemson.edu' 'billyzheng.bz@gmail.com',
    description='Particle Filter Localization using RangeLibc for Accelerated Ray Casting',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'particle_filter = particle_filter.particle_filter:main'
        ],
    },
)