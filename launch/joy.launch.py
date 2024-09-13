from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',  # Adjust this to your joystick device
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
                'coalesce_interval': 0.001,
            }]
        )
    ])