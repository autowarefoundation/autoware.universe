from setuptools import setup
import os
from glob import glob

package_name = 'autodrive_f1tenth'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')), # Map files
        (os.path.join('share', package_name, 'config/gym_rviz'), glob('config/gym_rviz/*')), # Config files for Gym RViz
        (os.path.join('share', package_name, 'config/simulator'), glob('config/simulator/*')), # Config files for Simulator
        (os.path.join('share', package_name, 'config/testbed'), glob('config/testbed/*')), # Config files for Testbed
        (os.path.join('share', package_name, 'launch/gym_rviz'), glob('launch/gym_rviz/*')), # Launch files for Gym RViz
        (os.path.join('share', package_name, 'launch/simulator'), glob('launch/simulator/*')), # Launch files for Simulator
        (os.path.join('share', package_name, 'launch/testbed'), glob('launch/testbed/*')), # Launch files for Testbed
        (os.path.join('share', package_name, 'rviz/gym_rviz'), glob('rviz/gym_rviz/*')), # RViz configuration files for Gym RViz
        (os.path.join('share', package_name, 'rviz/simulator'), glob('rviz/simulator/*')), # RViz configuration files for Simulator
        (os.path.join('share', package_name, 'rviz/testbed'), glob('rviz/testbed/*')), # RViz configuration files for Testbed
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chinmay Vilas Samak' 'Tanmay Vilas Samak',
    maintainer_email='csamak@clemson.edu' 'tsamak@clemson.edu',
    description='AutoDRIVE Ecosystem Autoware Integration for F1TENTH',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autodrive_incoming_bridge = autodrive_f1tenth.autodrive_incoming_bridge:main', # AutoDRIVE incoming bridge
            'autodrive_outgoing_bridge = autodrive_f1tenth.autodrive_outgoing_bridge:main', # AutoDRIVE outgoing bridge
            'teleop_keyboard = autodrive_f1tenth.teleop_keyboard:main', # Teleoperation with keyboard
        ],
    },
)