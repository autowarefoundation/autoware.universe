from setuptools import setup
import os
from glob import glob

package_name = 'autodrive_hunter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')), # Map files
        (os.path.join('share', package_name, 'config/simulator'), glob('config/simulator/*')), # Config files for AutoDRIVE Simulator
        (os.path.join('share', package_name, 'config/testbed'), glob('config/testbed/*')), # Config files for AutoDRIVE Testbed
        (os.path.join('share', package_name, 'launch/simulator'), glob('launch/simulator/*.launch.py')), # Launch files for AutoDRIVE Simulator
        (os.path.join('share', package_name, 'launch/testbed'), glob('launch/testbed/*.launch.py')), # Launch files for AutoDRIVE Testbed
        (os.path.join('share', package_name, 'rviz/simulator'), glob('rviz/simulator/*.rviz')), # RViz configuration files for AutoDRIVE Simulator
        (os.path.join('share', package_name, 'rviz/testbed'), glob('rviz/testbed/*.rviz')), # RViz configuration files for AutoDRIVE Testbed
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chinmay Vilas Samak' 'Tanmay Vilas Samak',
    maintainer_email='csamak@clemson.edu' 'tsamak@clemson.edu',
    description='AutoDRIVE Ecosystem Autoware Integration for Hunter SE',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autodrive_incoming_bridge = autodrive_hunter.autodrive_incoming_bridge:main', # AutoDRIVE incoming bridge
            'autodrive_outgoing_bridge = autodrive_hunter.autodrive_outgoing_bridge:main', # AutoDRIVE outgoing bridge
            'teleop_keyboard = autodrive_hunter.teleop_keyboard:main', # Teleoperation with keyboard
        ],
    },
)