from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
import os

def generate_launch_description():
    # Get the directory of the 'chienpanze_description' package
    pkg_dir = get_package_share_directory('chienpanze_description')
    
    # Define the path to the Xacro file
    xacro_path = os.path.join(pkg_dir, 'urdf', 'chienpanze.urdf.xacro')
    
    return LaunchDescription([
        # Node to convert Xacro to URDF and publish to '/robot_description'
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_path]), value_type=str)}],
        ),
        Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
            ),

        # Node to launch RViz
        ExecuteProcess(
            cmd=['rviz2'],
            output='screen'
        )
    ])


#later add:
# control_node = Node(
#     package="controller_manager",
#     executable="ros2_control_node",
#     parameters=[robot_controllers],
#     output="both",
#     namespace="rrbot",
# )