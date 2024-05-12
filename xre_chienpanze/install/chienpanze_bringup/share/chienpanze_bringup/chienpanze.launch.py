from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare  # Corrected import
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart

#import os
#from ament_index_python.packages import get_package_share_directory

from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Setup launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="chienpanze_description",
            description="Package with robot description (URDF)."
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="chienpanze.urdf.xacro",
            description="URDF/XACRO description file with the robot."
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="chienpanze_bringup",
            description="Package with the runtime configuration."
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="chienpanze_controllers.yaml",
            description="YAML file with the controller configurations."
        )
    ]

    # Configure robot description
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare(description_package), "urdf", description_file
            ])
        ]),
        value_type=str
    )

    # Controller configurations
    controllers_file = LaunchConfiguration("controllers_file")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", controllers_file
    ])

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[{'robot_description': robot_description_content}, robot_controllers]
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description_content}]
    )

    # Start the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Return the launch description
    return LaunchDescription(declared_arguments + [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner
    ])
