import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        launch.actions.LogInfo(
            msg="Launch icart_mini and ypspur node."
        ),
        Node(
            package='icart_mini_driver',
            namespace='icart_mini_driver',
            executable='icart_mini_driver_node',
        )
    ])