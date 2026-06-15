from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    urdf_path = LaunchConfiguration("urdf_file").perform(context)

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
        ),
    ]

def generate_launch_description():

    urdf_default = PathJoinSubstitution([
        FindPackageShare("robogami_description"),
        "urdf",
        "robogami.urdf"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_file",
            default_value=urdf_default,
            description="Path to URDF file"
        ),

        OpaqueFunction(function=launch_setup)
    ])