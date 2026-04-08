"""
real_robot.launch.py  –  Start MoveIt connected to the physical UR5e.

Before running:
  1. Set ROBOT_IP below to the UR5e's IP address (shown on the teach pendant
     under Settings → System → Network).
  2. On the teach pendant: load the External Control URCap program and make
     sure it is in REMOTE control mode (the key switch or the top-right toggle).
  3. Press PLAY on the teach pendant to start the External Control program.

Then in separate terminals:

    Terminal 1 – robot driver + MoveIt (this file):
        ros2 launch ur_cell_config real_robot.launch.py

    Terminal 2 – gcode executor:
        python3 ~/ur5e_ws_ian_testing/src/UR5e_3DP/gcode_executor_continuous.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


# ── Tuneable constants ────────────────────────────────────────────────────────
ROBOT_IP = "169.254.107.107"   # ← SET THIS to your UR5e's IP address
UR_TYPE  = "ur5e"
# ─────────────────────────────────────────────────────────────────────────────


def generate_launch_description():

    # ── 1. UR robot driver (connects to real hardware) ────────────────────────
    ur_control = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type":                  UR_TYPE,
            "robot_ip":                 ROBOT_IP,
            "launch_rviz":              "false",
            "launch_dashboard_client":  "false",
            "headless_mode":            "false",   # false = requires teach-pendant play
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            # Custom RSP launch: adds Vention stand, tool head, tool_tip to the real-hardware URDF
            "description_launchfile":   PathJoinSubstitution(
                [FindPackageShare("ur_cell_config"), "launch", "ur5e_real_rsp.launch.py"]
            ),
        }.items(),
    )

    # ── 2. MoveIt move_group  (ur_cell_config SRDF / kinematics) ─────────────
    move_group = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_cell_config"), "launch", "move_group.launch.py"]
            )
        ),
    )

    # ── 3. RViz ───────────────────────────────────────────────────────────────
    moveit_rviz = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_cell_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
    )

    # Give the driver ~5 s to connect before MoveIt tries to come up
    delayed_moveit = TimerAction(period=5.0, actions=[move_group, moveit_rviz])

    return LaunchDescription([
        ur_control,
        delayed_moveit,
    ])
