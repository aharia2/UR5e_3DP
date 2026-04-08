"""
ursim.launch.py  –  Start MoveIt connected to URSim (Docker).

This replaces demo.launch.py for simulation.  Run URSim first (see README),
then:

    Terminal 1 – robot driver (this file):
        ros2 launch ur_cell_config ursim.launch.py

    Terminal 2 – gcode executor:
        python3 ~/ros2_ws/UR5e_3DP/gcode_executor.py

The driver connects to URSim at 127.0.0.1 (localhost).
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


# ── Tuneable constants ────────────────────────────────────────────────────────
ROBOT_IP  = "127.0.0.1"   # URSim runs on localhost
UR_TYPE   = "ur5e"
# ─────────────────────────────────────────────────────────────────────────────


def generate_launch_description():

    # ── 1. UR robot driver (connects to URSim) ────────────────────────────────
    ur_control = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type":               UR_TYPE,
            "robot_ip":              ROBOT_IP,
            "launch_rviz":           "false",
            "launch_dashboard_client": "false",
            "headless_mode":         "true",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    # ── 2. MoveIt move_group  (your ur_cell_config SRDF / kinematics) ─────────
    move_group = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_cell_config"), "launch", "move_group.launch.py"]
            )
        ),
    )

    # ── 3. RViz (your MoveIt RViz config) ────────────────────────────────────
    moveit_rviz = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_cell_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
    )

    # Give the driver ~5 s to come up before MoveIt tries to connect
    delayed_moveit = TimerAction(period=5.0, actions=[move_group, moveit_rviz])

    return LaunchDescription([
        ur_control,
        delayed_moveit,
    ])
