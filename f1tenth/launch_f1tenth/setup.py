import  os
from glob import glob
from setuptools import setup

package_name = 'launch_autoware_f1tenth'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='autoware',
    maintainer_email='zhijie.qiao@autoware.org',
    description='launch the autoware on f1tenth demo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = launch_f1tenth.my_node:main'
        ],
    },
)
