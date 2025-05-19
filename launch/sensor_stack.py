from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the sensor node (this is the sensor mockup)
        Node(
            package='robotics_soln',
            executable='sensor',
            name='sensor',
            output='screen',
            emulate_tty=True,
        ),

        # Launch the sensor service node that filters the first sensor
        Node(
            package='robotics_soln',
            executable='sensor_service',
            name='sensor_service_0',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'addr': '127.0.0.1'}, # First Sensor Address
                {'port': 10000},  # Use default port
                {'id': 0}  # Sensor ID
            ]
        ),

        # Launch the sensor service node that filters the second sensor
        Node(
            package='robotics_soln',
            executable='sensor_service',
            name='sensor_service_1',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'addr': '127.0.0.3'}, # Second Sensor Address
                {'port': 10000},  # Use default port
                {'id': 1}  # Sensor ID
            ]
        ),

        # Launch the sensor client node that subscribes to the BOTH sensors
        Node(
            package='robotics_soln',
            executable='sensor_client',
            name='sensor_client',
            output='screen',
            emulate_tty=True,
        ),

        LogInfo(
            condition=None,
            msg="Launching sensor stack with two sensors and a client node."
        ),
    ])