#!/usr/bin/env python3
"""
klipper_delay_test.py
=====================
Tests Klipper extrusion timing relative to a 4-point robot Cartesian path.

The robot plans one continuous Cartesian path through four waypoints:

    START  →  EXT_START  →  EXT_END  →  END

A background thread fires a plain M82 + G1 E command to Moonraker at the
geometric EXT_START timestamp during trajectory execution — no G4 dwell in
Klipper.  KLIPPER_DELAY_SEC accounts for Moonraker HTTP + buffer latency
(command is sent that many seconds before the target waypoint).

Usage
-----
    python3 klipper_delay_test.py

Edit the parameters in the "User settings" section below, then run.
All positions are in metres relative to the UR5e base_link frame.
"""

import json
import math
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetCartesianPath


# ══════════════════════════════════════════════════════════════════════════════
# USER SETTINGS  —  edit everything in this block
# ══════════════════════════════════════════════════════════════════════════════

# ── Four waypoints (metres, base_link frame) ──────────────────────────────────

START_X,     START_Y,     START_Z     = -0.7,   -0.25, -0.532  # approach point
EXT_START_X, EXT_START_Y, EXT_START_Z = -0.675, -0.25, -0.532  # begin extrusion here
EXT_END_X,   EXT_END_Y,   EXT_END_Z   = -0.63,  -0.25, -0.532  # stop  extrusion here
END_X,       END_Y,       END_Z       = -0.58,  -0.25, -0.532  # retract / finish

# ── Nozzle orientation (quaternion) — same for all four points ────────────────
# Nozzle-down orientation matching initial_positions.yaml / real robot default.
ORIENT_QX =  0.5
ORIENT_QY = -0.5
ORIENT_QZ = -0.5
ORIENT_QW =  0.5

# ── Motion parameters ─────────────────────────────────────────────────────────

# Robot TCP movement speed (mm/min).  Converted to velocity_scale internally
# using MAX_SPEED_MM_PER_MIN = 60000.0 (same value as gcode_executorOLD.py).
MOVEMENT_SPEED_MM_PER_MIN = 10000.0

# Fraction of the robot's maximum acceleration (0.0 – 1.0).
ACCELERATION_SCALE = 0.5

# Maximum TCP speed the robot is configured for at velocity_scale = 1.0.
# Must match the value in gcode_executorOLD.py.
MAX_SPEED_MM_PER_MIN = 60000.0

# ── Extrusion parameters ──────────────────────────────────────────────────────

# Relative filament distance to extrude (mm).  Uses M83 relative mode so this
# is always the actual amount extruded regardless of current E position.
EXTRUSION_DISTANCE_MM = 10.0

# When True, the feedrate is computed automatically so that the extrusion
# finishes exactly when the robot reaches EXT_END (based on the geometric
# travel time between EXT_START and EXT_END).  EXTRUSION_SCALE then acts as
# a flow multiplier: 1.0 = perfectly matched, >1.0 = faster, <1.0 = slower.
# When False, EXTRUSION_FEEDRATE_MM_PER_MIN is used directly.
AUTO_FEEDRATE = True

# Manual extruder feedrate (mm/min).  Only used when AUTO_FEEDRATE = False.
EXTRUSION_FEEDRATE_MM_PER_MIN = 200.0

# Retraction fired at EXT_END while the robot continues to END.
# Pulls filament back to cut off flow cleanly and prevent a blob at EXT_END.
# Set to 0.0 to disable.
RETRACTION_MM = 1
RETRACTION_FEEDRATE_MM_PER_MIN = 1000.0

# ── Klipper / Moonraker ───────────────────────────────────────────────────────

MOONRAKER_URL = "http://localhost:7125"

# Time (seconds) between sending the Klipper command and starting the extrusion
# segment — identical semantics to gcode_executorOLD.py.
# Increase if extrusion starts too late; decrease if it starts too early.
KLIPPER_DELAY_SEC = 0.38

# ── MoveIt tuning ─────────────────────────────────────────────────────────────

# Cartesian interpolation step (metres).  Smaller = smoother, slower to plan.
CARTESIAN_STEP = 0.001

# Jump threshold for GetCartesianPath.
CARTESIAN_JUMP_THRESHOLD = 4.0

# Minimum fraction of the Cartesian path that must be solved to accept the plan.
MIN_CARTESIAN_FRACTION = 0.95

# Planning service call timeout (seconds).
PLANNING_TIMEOUT = 30.0

# ══════════════════════════════════════════════════════════════════════════════
# END OF USER SETTINGS
# ══════════════════════════════════════════════════════════════════════════════


# ─── Klipper communication ────────────────────────────────────────────────────

class KlipperComm:
    """Send G-code to Klipper via the Moonraker HTTP API."""

    def __init__(self, url: str = MOONRAKER_URL):
        self.url = url.rstrip('/')

    def connect(self) -> bool:
        """Check that Moonraker is reachable and Klipper is ready."""
        try:
            with urllib.request.urlopen(f'{self.url}/printer/info', timeout=5) as r:
                info = json.loads(r.read())
            state = info.get('result', {}).get('state', 'unknown')
            print(f'[Klipper] Connected — Moonraker at {self.url}  (state: {state})')
            return True
        except Exception as e:
            print(f'[Klipper] WARNING: Could not reach Moonraker: {e}')
            return False

    def send_gcode(self, script: str) -> bool:
        """Send a multi-line G-code script to Klipper's buffer."""
        try:
            data = json.dumps({'script': script}).encode()
            req = urllib.request.Request(
                f'{self.url}/printer/gcode/script',
                data=data,
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            with urllib.request.urlopen(req, timeout=60) as r:
                r.read()
            return True
        except Exception as e:
            print(f'[Klipper] ERROR sending G-code: {e}')
            return False


# ─── Helpers ──────────────────────────────────────────────────────────────────

def _dist(ax, ay, az, bx, by, bz) -> float:
    """3-D Euclidean distance (metres)."""
    return math.sqrt((bx - ax)**2 + (by - ay)**2 + (bz - az)**2)


def _make_pose(x, y, z,
               qx=ORIENT_QX, qy=ORIENT_QY,
               qz=ORIENT_QZ, qw=ORIENT_QW) -> Pose:
    p = Pose()
    p.position.x    = x
    p.position.y    = y
    p.position.z    = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


def traj_duration(traj) -> float:
    """Return the planned duration of a RobotTrajectory in seconds."""
    pts = traj.joint_trajectory.points
    if not pts:
        return 0.0
    t = pts[-1].time_from_start
    return t.sec + t.nanosec * 1e-9


def resolve_feedrate(t_ext_duration: float) -> float:
    """
    Return the extruder feedrate (mm/min) to use for the G1 E command.

    When AUTO_FEEDRATE is True: feedrate = (EXTRUSION_DISTANCE_MM / t_ext_duration)
    * 60, so the filament finishes exactly as the robot reaches EXT_END.
    Increase EXTRUSION_DISTANCE_MM to increase flow.
    When AUTO_FEEDRATE is False: returns EXTRUSION_FEEDRATE_MM_PER_MIN directly.
    """
    if AUTO_FEEDRATE:
        if t_ext_duration < 1e-9:
            return EXTRUSION_FEEDRATE_MM_PER_MIN  # fallback
        return (EXTRUSION_DISTANCE_MM / t_ext_duration) * 60.0
    return EXTRUSION_FEEDRATE_MM_PER_MIN


def make_extrusion_gcode(feedrate: float) -> str:
    """
    Build the full extrusion script sent as one block to Klipper:
      M83              — relative extrusion mode
      G1 E+X F<feed>   — extrude
      G1 E-X F<retract> — retract immediately after (if RETRACTION_MM > 0)

    Sending as one script means Klipper queues retraction back-to-back with
    extrusion — no gap between them.
    """
    lines = [
        'M83',
        f'G1 E{EXTRUSION_DISTANCE_MM:.4f} F{feedrate:.2f}',
    ]
    if RETRACTION_MM > 0:
        lines.append(f'G1 E-{RETRACTION_MM:.2f} F{RETRACTION_FEEDRATE_MM_PER_MIN:.1f}')
    return '\n'.join(lines)


def send_klipper_and_wait(klipper: 'KlipperComm', feedrate: float) -> None:
    """
    Mirror of gcode_executorOLD pattern:
      1. Send extrusion G-code to Klipper in a background thread.
      2. Block for KLIPPER_DELAY_SEC so Klipper buffers the command.
    The caller then immediately starts the extrusion trajectory.
    """
    gcode = make_extrusion_gcode(feedrate)
    print(f'  [Klipper] Sending extrusion command  '
          f'(E{EXTRUSION_DISTANCE_MM:.2f} mm  F{feedrate:.1f} mm/min)')
    threading.Thread(target=klipper.send_gcode, args=(gcode,), daemon=True).start()
    if KLIPPER_DELAY_SEC > 0:
        time.sleep(KLIPPER_DELAY_SEC)


# ─── Main node ────────────────────────────────────────────────────────────────

class KlipperDelayTestNode(Node):

    def __init__(self):
        super().__init__('klipper_delay_test')

        self._cart_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')
        self._exec_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')
        self._display_pub = self.create_publisher(
            DisplayTrajectory, '/display_planned_path', 1)

        self.get_logger().info('Waiting for MoveIt services…')
        self._cart_client.wait_for_service()
        self._exec_client.wait_for_server()
        self.get_logger().info('MoveIt services ready.')

    # ── Planning ──────────────────────────────────────────────────────────────

    def plan_cartesian_path(self, poses: list,
                            start_state: RobotState = None):
        """
        Plan a Cartesian path through *poses* at MOVEMENT_SPEED_MM_PER_MIN /
        ACCELERATION_SCALE.  Returns a RobotTrajectory on success, None on failure.
        """
        velocity_scale = min(1.0, max(0.01, MOVEMENT_SPEED_MM_PER_MIN / MAX_SPEED_MM_PER_MIN))
        req = GetCartesianPath.Request()
        req.header.stamp     = self.get_clock().now().to_msg()
        req.header.frame_id  = 'base_link'
        req.group_name       = 'ur_arm'
        req.link_name        = 'tool_tip'
        req.waypoints        = poses
        req.max_step         = CARTESIAN_STEP
        req.jump_threshold   = CARTESIAN_JUMP_THRESHOLD
        req.avoid_collisions = True
        req.max_velocity_scaling_factor     = float(velocity_scale)
        req.max_acceleration_scaling_factor = float(ACCELERATION_SCALE)
        if start_state is not None:
            req.start_state = start_state

        future = self._cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=PLANNING_TIMEOUT)

        if not future.done():
            self.get_logger().warn('Cartesian planning timed out.')
            return None

        response = future.result()
        if response.fraction < MIN_CARTESIAN_FRACTION:
            self.get_logger().warn(
                f'Cartesian path only {response.fraction * 100:.1f}% complete '
                f'(minimum: {MIN_CARTESIAN_FRACTION * 100:.0f}%).')
            return None

        return response.solution

    # ── Execution ─────────────────────────────────────────────────────────────

    def execute_trajectory(self, trajectory, timeout_sec: float = 30.0) -> bool:
        """Execute a pre-planned RobotTrajectory.  Returns True on success."""
        pts = trajectory.joint_trajectory.points
        if pts:
            t = pts[-1].time_from_start
            traj_secs = t.sec + t.nanosec * 1e-9
            timeout_sec = max(timeout_sec, traj_secs * 3.0 + 5.0)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self._exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error('Execution: goal send timed out.')
            return False

        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error('Execution: goal rejected.')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error(f'Execution: timed out after {timeout_sec:.0f}s.')
            return False

        return True

    # ── Main flow ─────────────────────────────────────────────────────────────

    def run(self) -> None:

        # Build the four poses.
        pose_start     = _make_pose(START_X,     START_Y,     START_Z)
        pose_ext_start = _make_pose(EXT_START_X, EXT_START_Y, EXT_START_Z)
        pose_ext_end   = _make_pose(EXT_END_X,   EXT_END_Y,   EXT_END_Z)
        pose_end       = _make_pose(END_X,        END_Y,       END_Z)

        print(f"\n{'='*60}")
        print(' Klipper Delay Test — waypoints')
        print(f"{'='*60}")
        print(f'  START     : ({START_X:.4f}, {START_Y:.4f}, {START_Z:.4f})')
        print(f'  EXT_START : ({EXT_START_X:.4f}, {EXT_START_Y:.4f}, {EXT_START_Z:.4f})')
        print(f'  EXT_END   : ({EXT_END_X:.4f}, {EXT_END_Y:.4f}, {EXT_END_Z:.4f})')
        print(f'  END       : ({END_X:.4f}, {END_Y:.4f}, {END_Z:.4f})')
        velocity_scale = min(1.0, max(0.01, MOVEMENT_SPEED_MM_PER_MIN / MAX_SPEED_MM_PER_MIN))
        print(f'\n  MOVEMENT_SPEED      : {MOVEMENT_SPEED_MM_PER_MIN} mm/min  '
              f'(velocity_scale={velocity_scale:.4f})')
        print(f'  ACCELERATION_SCALE  : {ACCELERATION_SCALE}')
        print(f'  EXTRUSION_DISTANCE  : {EXTRUSION_DISTANCE_MM} mm')
        if AUTO_FEEDRATE:
            print(f'  FEEDRATE MODE       : auto  (E / segment_time × 60)')
        else:
            print(f'  FEEDRATE MODE       : manual  ({EXTRUSION_FEEDRATE_MM_PER_MIN} mm/min)')
        print(f'  KLIPPER_DELAY_SEC   : {KLIPPER_DELAY_SEC} s')
        print(f"{'='*60}")

        # ── Step 1: Plan approach to START ────────────────────────────────────
        print('\nStep 1: Planning approach to START…')
        approach_traj = self.plan_cartesian_path([pose_start], start_state=None)
        if approach_traj is None:
            self.get_logger().error('Approach plan failed. Aborting.')
            return
        print('  Approach planned OK.')

        # ── Step 2a: Plan travel  START → EXT_START ───────────────────────────
        print('\nStep 2: Planning segments…')

        last_ap = approach_traj.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name     = list(approach_traj.joint_trajectory.joint_names)
        start_state.joint_state.position = list(last_ap.positions)
        start_state.joint_state.velocity = list(last_ap.velocities) if last_ap.velocities else []

        travel_traj = self.plan_cartesian_path(
            [pose_start, pose_ext_start],
            start_state=start_state,
        )
        if travel_traj is None:
            self.get_logger().error('Travel segment plan failed. Aborting.')
            return
        travel_dur = traj_duration(travel_traj)
        print(f'  Travel  START→EXT_START : {travel_dur:.3f}s')

        # ── Step 2b: Plan extrusion  EXT_START → EXT_END ─────────────────────
        last_tr = travel_traj.joint_trajectory.points[-1]
        ext_state = RobotState()
        ext_state.joint_state.name     = list(travel_traj.joint_trajectory.joint_names)
        ext_state.joint_state.position = list(last_tr.positions)
        ext_state.joint_state.velocity = list(last_tr.velocities) if last_tr.velocities else []

        extrusion_traj = self.plan_cartesian_path(
            [pose_ext_start, pose_ext_end],
            start_state=ext_state,
        )
        if extrusion_traj is None:
            self.get_logger().error('Extrusion segment plan failed. Aborting.')
            return
        ext_dur = traj_duration(extrusion_traj)
        print(f'  Extrusion EXT_START→EXT_END : {ext_dur:.3f}s')

        # ── Step 2c: Plan tail  EXT_END → END ────────────────────────────────
        last_ext = extrusion_traj.joint_trajectory.points[-1]
        tail_state = RobotState()
        tail_state.joint_state.name     = list(extrusion_traj.joint_trajectory.joint_names)
        tail_state.joint_state.position = list(last_ext.positions)
        tail_state.joint_state.velocity = list(last_ext.velocities) if last_ext.velocities else []

        tail_traj = self.plan_cartesian_path(
            [pose_ext_end, pose_end],
            start_state=tail_state,
        )
        if tail_traj is None:
            self.get_logger().error('Tail segment plan failed. Aborting.')
            return
        tail_dur = traj_duration(tail_traj)
        print(f'  Tail      EXT_END→END       : {tail_dur:.3f}s')

        # ── Step 3: Feedrate from EXT_START→EXT_END duration only ────────────
        feedrate = resolve_feedrate(ext_dur)
        feedrate_src = f'auto  ({ext_dur:.3f}s)' if AUTO_FEEDRATE else 'manual'
        print(f'\n  Feedrate : {feedrate:.2f} mm/min  [{feedrate_src}]')
        print(f'\n  Klipper command:')
        for line in make_extrusion_gcode(feedrate).splitlines():
            print(f'    {line}')

        # Display in RViz.
        display = DisplayTrajectory()
        display.trajectory.append(travel_traj)
        display.trajectory.append(extrusion_traj)
        display.trajectory.append(tail_traj)
        pts0 = travel_traj.joint_trajectory.points
        display.trajectory_start.joint_state.name     = list(travel_traj.joint_trajectory.joint_names)
        display.trajectory_start.joint_state.position = list(pts0[0].positions) if pts0 else []
        self._display_pub.publish(display)

        # ── Connect to Klipper ────────────────────────────────────────────────
        klipper = KlipperComm()
        klipper_ready = klipper.connect()
        if not klipper_ready:
            print('\n  WARNING: Klipper not reachable — extrusion will be skipped.')

        # ── Confirm: move to approach ─────────────────────────────────────────
        print(f"\n{'='*60}")
        try:
            ans = input("Press ENTER to move to START position, or 'q' to quit: ")
        except EOFError:
            ans = ''
        if ans.strip().lower() == 'q':
            print('Aborted.')
            return

        print('\nStep 3: Moving to START position…')
        if not self.execute_trajectory(approach_traj):
            self.get_logger().error('Approach execution failed. Aborting.')
            return
        print('  At START position.')

        # ── Confirm: run test ─────────────────────────────────────────────────
        print(f"\n{'='*60}")
        try:
            ans = input("Press ENTER to run the extrusion delay test, or 'q' to quit: ")
        except EOFError:
            ans = ''
        if ans.strip().lower() == 'q':
            print('Aborted.')
            return

        # ── Step 4: Execute travel, then send klipper + execute extrusion ────
        print(f"\n{'='*60}")
        print(' Step 4: Running test')
        print(f"{'='*60}")

        print('  Executing travel segment  START → EXT_START…')
        if not self.execute_trajectory(travel_traj):
            self.get_logger().error('Travel execution failed. Aborting.')
            return
        print('  At EXT_START.')

        # Executor pattern: send klipper, sleep KLIPPER_DELAY_SEC, execute.
        if klipper_ready:
            send_klipper_and_wait(klipper, feedrate)
        else:
            print('  [Klipper] Extrusion skipped (not connected).')

        print('  Executing extrusion segment  EXT_START → EXT_END…')
        if not self.execute_trajectory(extrusion_traj):
            self.get_logger().error('Extrusion trajectory execution failed.')
            return
        print('  At EXT_END — extrusion complete.')

        print('  Executing tail  EXT_END → END…')
        ok = self.execute_trajectory(tail_traj)

        if ok:
            print('\n  Test complete — check extrusion result.')
        else:
            self.get_logger().error('Tail trajectory execution failed.')

        print(f"{'='*60}\n")


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = KlipperDelayTestNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
