# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess
# from launch.substitutions import Command, PathJoinSubstitution
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.descriptions import ParameterValue
# import os

# def generate_launch_description():
#     # Get the directory of the 'chienpanze_description' package
#     pkg_dir = get_package_share_directory('chienpanze_description')
    
#     # Define the path to the Xacro file
#     xacro_path = os.path.join(pkg_dir, 'urdf', 'chienpanze.urdf.xacro')
    
#     return LaunchDescription([
#         # Node to convert Xacro to URDF and publish to '/robot_description'
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='both',
#             parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_path]), value_type=str)}],
#         ),
#         Node(
#                 package="joint_state_publisher",
#                 executable="joint_state_publisher",
#                 name="joint_state_publisher",
#             ),

#         # Node to launch RViz
#         ExecuteProcess(
#             cmd=['rviz2'],
#             output='screen'
#         )
#     ])


#later add:
# control_node = Node(
#     package="controller_manager",
#     executable="ros2_control_node",
#     parameters=[robot_controllers],
#     output="both",
#     namespace="chienpanze",
# )




from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="chienpanze_description",
            description="Description package of the chienpanze. Usually the argument is not set, \
        it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "chienpanze.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "chienpanze.rviz"]
    )

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
        arguments=["-d", rviz_config_file],
    )

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