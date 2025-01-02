from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motors',
            executable='test_sensors',
            name='UI_TestingNode', 
        ),
        Node(
            package='motors',
            executable='service_test',
            name='SensorInstallationService',
        ),
        # Rosbridge server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge',
        ),
    ])
