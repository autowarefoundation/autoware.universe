from setuptools import setup

package_name = 'grpc_ros2_bridge_test_automation'
clients = 'grpc_ros2_bridge_test_automation/clients'
publishers = 'grpc_ros2_bridge_test_automation/publishers'
subscribers = 'grpc_ros2_bridge_test_automation/subscribers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, clients, publishers, subscribers],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shpark',
    maintainer_email='shpark@morai.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Sim Setting
            'ctrl_mode = grpc_ros2_bridge_test_automation.clients.client_event_cmd:main',
            # Publisher
            'gear_command = grpc_ros2_bridge_test_automation.publishers.gear_command:main',
            'control_mode_command = grpc_ros2_bridge_test_automation.publishers.control_mode_command:main',
            # Subscriber
            'gear_report = grpc_ros2_bridge_test_automation.subscribers.gear_report:main',
            'control_mode_report = grpc_ros2_bridge_test_automation.subscribers.control_mode_report:main',
            'velocity_report = grpc_ros2_bridge_test_automation.subscribers.velocity_report:main',
            'steering_report = grpc_ros2_bridge_test_automation.subscribers.steering_report:main',
            'turn_indicators_report = grpc_ros2_bridge_test_automation.subscribers.turn_indicators_report:main',
            'hazard_lights_report = grpc_ros2_bridge_test_automation.subscribers.hazard_lights_report:main'
        ],
    },
)
