"""
ur5e_real_rsp.launch.py
=======================
Robot State Publisher for the PHYSICAL UR5e.

Passed to ur_control.launch.py as `description_launchfile`.
Uses ur5e_real.urdf.xacro which adds the Vention stand, tool head,
and tool_tip frame on top of the real UR hardware interface.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type   = LaunchConfiguration("ur_type")
    robot_ip  = LaunchConfiguration("robot_ip")

    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur5e_description"), "urdf", "ur5e_real.urdf.xacro"]
            ),
            " robot_ip:=",          robot_ip,
            " ur_type:=",           ur_type,
            " name:=",              ur_type,
            " script_filename:=",   script_filename,
            " input_recipe_filename:=",  input_recipe_filename,
            " output_recipe_filename:=", output_recipe_filename,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument("ur_type",  description="UR robot type"),
        DeclareLaunchArgument("robot_ip", description="IP address of the UR5e"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),
    ])
