
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('chienpanze_bringup'),
        'config',
        'motor_controllers.yaml'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_path],
        output='screen',
    )

    return LaunchDescription([
        controller_manager_node
    ])
