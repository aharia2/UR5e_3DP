#!/usr/bin/env python3
"""
jog_nozzle.py  –  Move the UR5e nozzle (tool_tip) to a target position via
                  a Cartesian path.

Edit TARGET_X/Y/Z below, then run:
    python3 jog_nozzle.py

Coordinates are in metres relative to the UR5e base_link.
Use this to jog the nozzle to the bed to find the bed offset.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import DisplayTrajectory


# ── Target position — edit these ──────────────────────────────────────────────

TARGET_X = -0.7# metres
TARGET_Y = -0.25      # metres
TARGET_Z = -.5315 # metres

# Nozzle-down orientation (matches initial_positions.yaml / real robot)
TARGET_QX = 0.5
TARGET_QY = -0.5
TARGET_QZ = -0.5    #edit this one to change the orientation about the z axis
TARGET_QW = 0.5

# Speed as a fraction of the robot's maximum (0.0 – 1.0)
VELOCITY_SCALE = 0.05

# ─────────────────────────────────────────────────────────────────────────────


class JogNode(Node):

    def __init__(self):
        super().__init__('jog_nozzle')
        self._cart_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')
        self._exec_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')
        self._display_pub = self.create_publisher(
            DisplayTrajectory, '/display_planned_path', 1)

    def jog(self):
        self.get_logger().info('Waiting for MoveIt services…')
        self._cart_client.wait_for_service()
        self._exec_client.wait_for_server()
        self.get_logger().info('MoveIt services ready.')

        target_pose = Pose()
        target_pose.position.x    = TARGET_X
        target_pose.position.y    = TARGET_Y
        target_pose.position.z    = TARGET_Z
        target_pose.orientation.x = TARGET_QX
        target_pose.orientation.y = TARGET_QY
        target_pose.orientation.z = TARGET_QZ
        target_pose.orientation.w = TARGET_QW

        # ── Plan Cartesian path ───────────────────────────────────────────────
        req = GetCartesianPath.Request()
        req.header.stamp     = self.get_clock().now().to_msg()
        req.header.frame_id  = 'base_link'
        req.group_name       = 'ur_arm'
        req.link_name        = 'tool_tip'
        req.waypoints        = [target_pose]
        req.max_step         = 0.001
        req.jump_threshold   = 4.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor     = VELOCITY_SCALE
        req.max_acceleration_scaling_factor = VELOCITY_SCALE

        self.get_logger().info(
            f'Planning Cartesian path to ({TARGET_X}, {TARGET_Y}, {TARGET_Z})…')

        future = self._cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done() or future.result() is None:
            self.get_logger().error('Cartesian planning timed out.')
            return

        response = future.result()
        if response.fraction < 0.95:
            self.get_logger().error(
                f'Cartesian path only {response.fraction*100:.1f}% complete — aborting.')
            return

        # ── Display in RViz ──────────────────────────────────────────────────
        display = DisplayTrajectory()
        display.trajectory.append(response.solution)
        display.trajectory_start.joint_state.name     = list(response.solution.joint_trajectory.joint_names)
        display.trajectory_start.joint_state.position = list(response.solution.joint_trajectory.points[0].positions)
        self._display_pub.publish(display)
        print(f'\nPlan complete ({response.fraction*100:.1f}% of path).')
        print('Check RViz to preview the motion.')

        try:
            ans = input('Press ENTER to execute, or q to abort: ')
        except EOFError:
            ans = ''
        if ans.strip().lower() == 'q':
            print('Aborted.')
            return

        # ── Execute ───────────────────────────────────────────────────────────
        pts = response.solution.joint_trajectory.points
        traj_secs = 0.0
        if pts:
            t = pts[-1].time_from_start
            traj_secs = t.sec + t.nanosec * 1e-9
        timeout_sec = max(30.0, traj_secs * 3.0 + 5.0)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution

        send_future = self._exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error('Execution: goal send timed out.')
            return

        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error('Execution: goal rejected.')
            return

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error('Execution: timed out.')
            return

        self.get_logger().info('Done — nozzle at target.')


def main():
    rclpy.init()
    node = JogNode()
    node.jog()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
