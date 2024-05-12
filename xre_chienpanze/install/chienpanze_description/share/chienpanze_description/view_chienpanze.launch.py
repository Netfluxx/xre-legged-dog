from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="chienpanze_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix for joint names, useful for multi-robot setups.",
        )
    ]

    # Get configurations
    description_package = LaunchConfiguration("description_package")
    prefix = LaunchConfiguration("prefix")

    # Get URDF from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "chienpanze.urdf.xacro"]
            ),
            " ",
            "prefix:=", prefix
        ]
    )

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Nodes
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare(description_package), "rviz", "chienpanze.rviz"]
        )],
    )

    # Event handlers for delays
    delay_rviz_after_joint_state_publisher_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_publisher_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[rviz_node],
                ),
            ],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_node,
            robot_state_publisher_node,
            delay_rviz_after_joint_state_publisher_node,
        ]
    )
