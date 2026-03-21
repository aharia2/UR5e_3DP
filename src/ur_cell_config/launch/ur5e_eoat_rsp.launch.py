"""
ur5e_eoat_rsp.launch.py

Custom robot_state_publisher launch for ur_control.launch.py.
Identical to ur_robot_driver/launch/ur_rsp.launch.py but hardcodes
description_file to ur5e_description/urdf/ur5e_with_eoat.urdf.xacro,
which wraps the standard UR driver xacro and adds the 3D-printing EOAT.

Pass this file as the description_launchfile argument to ur_control.launch.py.
"""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    physical_params_file = LaunchConfiguration("physical_params_file")
    visual_params_file = LaunchConfiguration("visual_params_file")
    joint_limit_params_file = LaunchConfiguration("joint_limit_params_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")

    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    # ── Our custom xacro: standard UR driver URDF + tool_head + tool_tip ──
    description_file = PathJoinSubstitution(
        [FindPackageShare("ur5e_description"), "urdf", "ur5e_with_eoat.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " robot_ip:=", robot_ip,
            " joint_limit_params:=", joint_limit_params_file,
            " kinematics_params:=", kinematics_params_file,
            " physical_params:=", physical_params_file,
            " visual_params:=", visual_params_file,
            " safety_limits:=", safety_limits,
            " safety_pos_margin:=", safety_pos_margin,
            " safety_k_position:=", safety_k_position,
            " name:=", ur_type,
            " script_filename:=", script_filename,
            " input_recipe_filename:=", input_recipe_filename,
            " output_recipe_filename:=", output_recipe_filename,
            " tf_prefix:=", tf_prefix,
            " use_mock_hardware:=", use_mock_hardware,
            " mock_sensor_commands:=", mock_sensor_commands,
            " headless_mode:=", headless_mode,
            " use_tool_communication:=", use_tool_communication,
            " tool_parity:=", tool_parity,
            " tool_baud_rate:=", tool_baud_rate,
            " tool_stop_bits:=", tool_stop_bits,
            " tool_rx_idle_chars:=", tool_rx_idle_chars,
            " tool_tx_idle_chars:=", tool_tx_idle_chars,
            " tool_device_name:=", tool_device_name,
            " tool_tcp_port:=", tool_tcp_port,
            " tool_voltage:=", tool_voltage,
            " reverse_ip:=", reverse_ip,
            " script_command_port:=", script_command_port,
            " reverse_port:=", reverse_port,
            " script_sender_port:=", script_sender_port,
            " trajectory_port:=", trajectory_port,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    declared_arguments = [
        DeclareLaunchArgument("ur_type", description="UR robot type (e.g. ur5e)."),
        DeclareLaunchArgument("robot_ip", description="IP of the robot / URSim."),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument(
            "joint_limit_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "physical_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "visual_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
            ),
        ),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("mock_sensor_commands", default_value="false"),
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("use_tool_communication", default_value="false"),
        DeclareLaunchArgument("tool_parity", default_value="0"),
        DeclareLaunchArgument("tool_baud_rate", default_value="115200"),
        DeclareLaunchArgument("tool_stop_bits", default_value="1"),
        DeclareLaunchArgument("tool_rx_idle_chars", default_value="1.5"),
        DeclareLaunchArgument("tool_tx_idle_chars", default_value="3.5"),
        DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR"),
        DeclareLaunchArgument("tool_tcp_port", default_value="54321"),
        DeclareLaunchArgument("tool_voltage", default_value="0"),
        DeclareLaunchArgument("reverse_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("script_command_port", default_value="50004"),
        DeclareLaunchArgument("reverse_port", default_value="50001"),
        DeclareLaunchArgument("script_sender_port", default_value="50002"),
        DeclareLaunchArgument("trajectory_port", default_value="50003"),
    ]

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
        ]
    )
