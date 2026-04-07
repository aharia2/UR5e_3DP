"""
robot.launch.py  –  Start MoveIt connected to a real UR5e robot.

Usage:
    ros2 launch ur_cell_config robot.launch.py robot_ip:=<ROBOT_IP>

Example:
    ros2 launch ur_cell_config robot.launch.py robot_ip:=192.168.1.100

Make sure:
  1. The robot is powered on and in Remote Control mode.
  2. The External Control URCap program is installed and running on the teach pendant.
  3. Your PC can ping the robot IP before launching.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


UR_TYPE = "ur5e"


def generate_launch_description():

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        description="IP address of the real UR5e robot (e.g. 192.168.1.100)",
    )
    robot_ip = LaunchConfiguration("robot_ip")

    # ── 1. UR robot driver (connects to real robot) ───────────────────────
    ur_control = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type":                  UR_TYPE,
            "robot_ip":                 robot_ip,
            "launch_rviz":              "false",
            "launch_dashboard_client":  "false",
            "headless_mode":            "false",   # teach pendant stays active
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            # Custom RSP that includes tool_head + tool_tip EOAT
            "description_launchfile": PathJoinSubstitution(
                [FindPackageShare("ur_cell_config"), "launch", "ur5e_eoat_rsp.launch.py"]
            ),
        }.items(),
    )

    # ── 2. MoveIt move_group ──────────────────────────────────────────────
    move_group = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_cell_config"), "launch", "move_group.launch.py"]
            )
        ),
    )

    # ── 3. RViz ───────────────────────────────────────────────────────────
    moveit_rviz = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_cell_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
    )

    # Give the driver 5 s to connect before MoveIt starts
    delayed_moveit = TimerAction(period=5.0, actions=[move_group, moveit_rviz])

    return LaunchDescription([
        robot_ip_arg,
        ur_control,
        delayed_moveit,
    ])
