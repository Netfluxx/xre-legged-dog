from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.descriptions import ParameterValue  # Make sure to import ParameterValue

def generate_launch_description():
    # Configure robot description
    description_package = "chienpanze_description"
    description_file = "chienpanze.urdf.xacro"
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

    robot_description = {'robot_description': robot_description_content}

    # Controller configurations
    runtime_config_package = "chienpanze_bringup"
    controllers_file = "chienpanze_controllers.yaml"
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", controllers_file
    ])

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, robot_controllers]
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    # Start the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Setup the spawner for the position_controller
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    # Return the launch description
    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        position_controller_spawner
    ])
