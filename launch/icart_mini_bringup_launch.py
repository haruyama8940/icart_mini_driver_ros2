import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    icart_mini_driver_dir = get_package_share_directory('icart_mini_driver')
    icart_mini_param = os.path.join(icart_mini_driver_dir,'config','icart_mini.param.yaml')
    return LaunchDescription([
        launch.actions.LogInfo(
            msg="Launch ypspur coordinator."
        ),
        # Node(
        #     package='icart_mini_driver',
        #     namespace='ypspur_coordinator',
        #     executable='ypspur_coordinator_bridge',
        #     parameters=[icart_mini_param]
        # ),
         launch.actions.LogInfo(
            msg="Launch icart_mini_mini_driver node."
        ),
        Node(
            package='icart_mini_driver',
            namespace='icart_mini_driver',
            executable='icart_mini_driver',
        )
    ])