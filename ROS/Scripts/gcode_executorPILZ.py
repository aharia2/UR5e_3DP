#!/usr/bin/env python3
"""
gcode_executorPILZ.py
=====================
Reads the CSV output of gcode_interpreter.py and executes the print path on
the UR5e via MoveIt's PILZ Industrial Motion Planner (LIN motion type).

Unlike gcode_executorOLD.py (which uses GetCartesianPath with joint-space time
parameterisation), this script uses PILZ LIN which enforces a constant
Cartesian TCP speed.  velocity_scale is now a true fraction of the Cartesian
maximum translational velocity defined in pilz_cartesian_limits.yaml
(max_trans_vel = 1.0 m/s → MAX_SPEED_MM_PER_MIN = 60 000 mm/min).

Each segment is planned as a series of point-to-point PILZ LIN moves (one per
consecutive waypoint pair) whose resulting trajectories are concatenated into
a single RobotTrajectory before execution.

Usage
-----
  python3 gcode_executorPILZ.py <interpreted_gcode.csv>

Example
-------
  python3 gcode_executorPILZ.py "gcode_interpreted files/40x40x40.csv"
"""

import concurrent.futures
import copy
import csv
import math
import os
import sys
import threading
import time
import urllib.parse
import urllib.request
import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    RobotState, Constraints,
    PositionConstraint, OrientationConstraint,
)
from shape_msgs.msg import SolidPrimitive

# ─── User settings ────────────────────────────────────────────────────────────

# Maximum TCP speed at velocity_scale = 1.0  (mm/min).
# Must match pilz_cartesian_limits.yaml  max_trans_vel (1.0 m/s = 60 000 mm/min).
# With PILZ LIN this is a true Cartesian TCP speed limit, not a joint limit.
MAX_SPEED_MM_PER_MIN = 60000.0

# Acceleration scaling factor applied to all moves (0.0 – 1.0).
ACCELERATION_SCALE = 0.3

# Timeout (seconds) allowed for each individual PILZ LIN planning call.
PLANNING_TIMEOUT = 30.0

# MoveIt2 motion planning service.  move_group routes to PILZ based on
# planner_id = 'LIN' set in each request — no separate per-pipeline endpoint.
PILZ_SERVICE = '/plan_kinematic_path'

# ─── Klipper / Moonraker settings ─────────────────────────────────────────────

MOONRAKER_URL = "http://localhost:7125"

# Fixed delay (seconds) inserted at the start of the extrusion script so that
# Klipper's buffered commands stay in sync with the joint trajectory.
KLIPPER_DELAY_SEC = 0.36

# ──────────────────────────────────────────────────────────────────────────────


# ─── CSV loading ──────────────────────────────────────────────────────────────

def load_csv(path: str) -> list:
    """
    Load a gcode_interpreter.py CSV output.

    CSV columns  (from gcode_interpreter.py):
      X, Y, Z, QX, QY, QZ, QW,
      Toolhead_Speed_mm_per_min,
      Extrusion_Length_mm,
      Extrusion_Speed_mm_per_min,
      Move_Type, Segment_ID, GCode_Line

    Returns a list of waypoint dicts with keys:
      x, y, z          – metres  (converted from mm)
      qx, qy, qz, qw  – tool orientation quaternion
      speed            – Toolhead_Speed in mm/min
      extrusion_mm     – Extrusion_Length_mm (filament delta)
      extrusion_speed  – Extrusion_Speed_mm_per_min
      move_type        – 'extrusion' or 'travel'
      segment_id       – integer group ID
      gcode_line       – source line in the original G-code file
    """
    waypoints = []
    with open(path, 'r') as fh:
        for raw_row in csv.reader(fh):
            row = [c.strip() for c in raw_row]
            if not row or row[0].startswith('#'):
                continue
            if row[0].upper() == 'X':
                continue  # header row
            if len(row) < 13:
                continue
            try:
                waypoints.append({
                    'x':             float(row[0]) * 0.001,  # mm → m
                    'y':             float(row[1]) * 0.001,
                    'z':             float(row[2]) * 0.001,
                    'qx':            float(row[3]),
                    'qy':            float(row[4]),
                    'qz':            float(row[5]),
                    'qw':            float(row[6]),
                    'speed':         float(row[7]),
                    'extrusion_mm':  float(row[8]),
                    'extrusion_speed': float(row[9]),
                    'move_type':     row[10],
                    'segment_id':    int(float(row[11])),
                    'gcode_line':    int(float(row[12])),
                })
            except (ValueError, IndexError):
                pass  # skip malformed rows
    return waypoints


def group_by_segment(waypoints: list) -> list:
    """
    Group waypoints by their Segment_ID (assigned by gcode_interpreter).

    Returns a list of segment dicts:
      segment_id        – int
      move_type         – 'extrusion' or 'travel'
      waypoints         – list of waypoint dicts belonging to this segment
      speed_mm_per_min  – toolhead speed (same for all waypoints in segment)
      velocity_scale    – speed / MAX_SPEED_MM_PER_MIN, clamped to [0.01, 1.0]
      total_extrusion   – sum of extrusion_mm across all waypoints in segment
    """
    if not waypoints:
        return []

    segments = []
    cur_id  = waypoints[0]['segment_id']
    cur_wps = [waypoints[0]]

    for wp in waypoints[1:]:
        if wp['segment_id'] == cur_id:
            cur_wps.append(wp)
        else:
            segments.append(_build_segment(cur_id, cur_wps))
            cur_id  = wp['segment_id']
            cur_wps = [wp]

    segments.append(_build_segment(cur_id, cur_wps))
    return segments


def _build_segment(seg_id: int, wps: list) -> dict:
    speed = wps[0]['speed']
    return {
        'segment_id':      seg_id,
        'move_type':       wps[0]['move_type'],
        'waypoints':       wps,
        'speed_mm_per_min': speed,
        'velocity_scale':  min(1.0, max(0.01, speed / MAX_SPEED_MM_PER_MIN)),
        'total_extrusion': sum(wp['extrusion_mm'] for wp in wps),
    }


def _make_pose(wp: dict) -> Pose:
    pose = Pose()
    pose.position.x    = wp['x']
    pose.position.y    = wp['y']
    pose.position.z    = wp['z']
    pose.orientation.x = wp['qx']
    pose.orientation.y = wp['qy']
    pose.orientation.z = wp['qz']
    pose.orientation.w = wp['qw']
    return pose


# ════════════════════════════════════════════════════════════════════════════════
# EXTRUSION LOGIC
# ════════════════════════════════════════════════════════════════════════════════

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


def _segment_waypoint_timestamps(seg: dict) -> list:
    """
    Compute the time (seconds, relative to segment start) at which the
    toolhead reaches each waypoint, using cumulative 3D path distance and
    the segment's constant toolhead speed.

    Returns a list of floats, one per waypoint (first entry is always 0.0).
    """
    wps = seg['waypoints']
    speed_m_per_s = seg['speed_mm_per_min'] / 60000.0  # mm/min → m/s
    timestamps = [0.0]
    for i in range(1, len(wps)):
        prev, cur = wps[i - 1], wps[i]
        dx = cur['x'] - prev['x']
        dy = cur['y'] - prev['y']
        dz = cur['z'] - prev['z']
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)  # metres
        dt = dist / speed_m_per_s if speed_m_per_s > 1e-9 else 0.0
        timestamps.append(timestamps[-1] + dt)
    return timestamps


def build_extrusion_schedule(segments: list, planned: list) -> list:
    """
    Build the global extrusion schedule as a list of (timestamp_sec, gcode) pairs.
    """
    schedule = []
    cumulative_e = 0.0
    global_seg_start = 0.0

    for seg, traj in zip(segments, planned):
        seg_duration = 0.0
        if traj is not None:
            pts = traj.joint_trajectory.points
            if pts:
                t = pts[-1].time_from_start
                seg_duration = t.sec + t.nanosec * 1e-9

        if seg['move_type'] == 'extrusion':
            wp_times = _segment_waypoint_timestamps(seg)
            wps = seg['waypoints']
            for j in range(1, len(wps)):
                e_delta = wps[j]['extrusion_mm']
                if e_delta < 1e-9:
                    continue
                cumulative_e += e_delta
                ext_speed = wps[j]['extrusion_speed']
                t_stamp = global_seg_start + wp_times[j - 1]
                schedule.append((t_stamp, f'G1 E{cumulative_e:.4f} F{ext_speed:.4f}'))
        else:
            for wp in seg['waypoints']:
                cumulative_e += wp['extrusion_mm']

        global_seg_start += seg_duration

    return schedule


def schedule_to_gcode(schedule: list) -> str:
    """
    Convert a list of (timestamp_sec, gcode) pairs into a single G-code script
    using G4 P<ms> dwell commands to reproduce the timing.
    """
    lines = ['M82']
    prev_t = 0.0
    for t, cmd in schedule:
        dt_ms = max(0, int((t - prev_t) * 1000))
        if dt_ms > 0:
            lines.append(f'G4 P{dt_ms}')
        lines.append(cmd)
        prev_t = t
    return '\n'.join(lines)


# ════════════════════════════════════════════════════════════════════════════════
# PILZ LIN PLANNING HELPERS
# ════════════════════════════════════════════════════════════════════════════════

def _build_lin_request(node: Node, target_pose: Pose, velocity_scale: float,
                       start_state: RobotState = None) -> GetMotionPlan.Request:
    """Build a GetMotionPlan request for a single PILZ LIN move."""
    req = GetMotionPlan.Request()
    mpr = req.motion_plan_request
    mpr.group_name              = 'ur_arm'
    mpr.pipeline_id             = 'pilz_industrial_motion_planner'
    mpr.planner_id              = 'LIN'
    mpr.num_planning_attempts   = 1
    mpr.allowed_planning_time   = PLANNING_TIMEOUT
    mpr.max_velocity_scaling_factor     = float(velocity_scale)
    mpr.max_acceleration_scaling_factor = float(ACCELERATION_SCALE)

    if start_state is not None:
        mpr.start_state = start_state

    # Position constraint — 1 mm tolerance sphere centred at the target.
    pos = PositionConstraint()
    pos.header.frame_id = 'base_link'
    pos.header.stamp    = node.get_clock().now().to_msg()
    pos.link_name       = 'tool_tip'
    sphere = SolidPrimitive()
    sphere.type         = SolidPrimitive.SPHERE
    sphere.dimensions   = [0.001]
    pos.constraint_region.primitives     = [sphere]
    pos.constraint_region.primitive_poses = [target_pose]
    pos.weight = 1.0

    # Orientation constraint — tight tolerance to maintain tool orientation.
    ori = OrientationConstraint()
    ori.header.frame_id = 'base_link'
    ori.header.stamp    = node.get_clock().now().to_msg()
    ori.link_name       = 'tool_tip'
    ori.orientation     = target_pose.orientation
    ori.absolute_x_axis_tolerance = 0.001
    ori.absolute_y_axis_tolerance = 0.001
    ori.absolute_z_axis_tolerance = 0.001
    ori.weight = 1.0

    goal = Constraints()
    goal.position_constraints    = [pos]
    goal.orientation_constraints = [ori]
    mpr.goal_constraints = [goal]

    return req


def _extract_end_state(traj) -> RobotState:
    """Build a RobotState from the last point of a RobotTrajectory."""
    last_pt = traj.joint_trajectory.points[-1]
    rs = RobotState()
    rs.joint_state.name     = list(traj.joint_trajectory.joint_names)
    rs.joint_state.position = list(last_pt.positions)
    rs.joint_state.velocity = list(last_pt.velocities) if last_pt.velocities else []
    return rs


def _concatenate_trajectories(trajs: list):
    """Concatenate a list of RobotTrajectory objects into one, offsetting timestamps."""
    combined = copy.deepcopy(trajs[0])
    for traj in trajs[1:]:
        if not combined.joint_trajectory.points:
            combined = copy.deepcopy(traj)
            continue
        last = combined.joint_trajectory.points[-1].time_from_start
        offset = last.sec + last.nanosec * 1e-9
        for pt in traj.joint_trajectory.points[1:]:  # skip first — duplicate of last point
            pt_new = copy.deepcopy(pt)
            t = pt_new.time_from_start.sec + pt_new.time_from_start.nanosec * 1e-9
            t_total = t + offset
            pt_new.time_from_start.sec      = int(t_total)
            pt_new.time_from_start.nanosec  = int(round((t_total - int(t_total)) * 1e9))
            combined.joint_trajectory.points.append(pt_new)
    return combined


def _plan_one_lin_threadsafe(node: Node, client, start_state, target_pose: Pose,
                             velocity_scale: float):
    """
    Plan a single PILZ LIN move from start_state to target_pose.

    Thread-safe: uses an Event to wait for the async ROS callback without
    blocking the ROS executor thread.  Returns a RobotTrajectory or None.
    """
    req   = _build_lin_request(node, target_pose, velocity_scale, start_state)
    event = threading.Event()
    result_holder = [None]

    def _cb(future):
        try:
            resp = future.result()
            if resp.motion_plan_response.error_code.val == 1:  # SUCCESS
                result_holder[0] = resp.motion_plan_response.trajectory
        except Exception:
            pass
        event.set()

    ros_fut = client.call_async(req)
    ros_fut.add_done_callback(_cb)
    event.wait(timeout=PLANNING_TIMEOUT + 5.0)
    return result_holder[0]


def _plan_lin_segment_threadsafe(node: Node, client, poses: list,
                                 velocity_scale: float, start_state=None):
    """
    Plan a full segment as sequential PILZ LIN moves (one per pose).
    Safe to call from a background thread.
    Returns a concatenated RobotTrajectory, or None on any failure.
    """
    if not poses:
        return None

    trajs     = []
    cur_state = start_state

    for target_pose in poses:
        traj = _plan_one_lin_threadsafe(node, client, cur_state, target_pose, velocity_scale)
        if traj is None:
            return None
        trajs.append(traj)
        cur_state = _extract_end_state(traj)

    if len(trajs) == 1:
        return trajs[0]
    return _concatenate_trajectories(trajs)


# ════════════════════════════════════════════════════════════════════════════════
# BACKGROUND PLANNER
# ════════════════════════════════════════════════════════════════════════════════

class _BackgroundPlanner:
    """
    Dedicated ROS2 node that runs PILZ LIN segment planning in a background
    thread pool, allowing the main thread to overlap planning of segment N+1
    with execution of segment N.
    """

    def __init__(self):
        self._node   = rclpy.create_node('gcode_planner_bg')
        self._client = self._node.create_client(GetMotionPlan, PILZ_SERVICE)
        self._ros_exec = rclpy.executors.SingleThreadedExecutor()
        self._ros_exec.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._ros_exec.spin, daemon=True)
        self._spin_thread.start()
        self._client.wait_for_service()
        self._pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)

    def submit(self, poses: list, velocity_scale: float,
               start_state=None) -> concurrent.futures.Future:
        """
        Non-blocking: submit a segment planning request.
        Returns a concurrent.futures.Future that resolves to a
        RobotTrajectory on success, or None on failure.
        """
        py_fut = concurrent.futures.Future()

        def _do_plan():
            try:
                result = _plan_lin_segment_threadsafe(
                    self._node, self._client, poses, velocity_scale, start_state)
                py_fut.set_result(result)
            except Exception:
                py_fut.set_result(None)

        self._pool.submit(_do_plan)
        return py_fut

    def shutdown(self):
        self._pool.shutdown(wait=False)
        self._ros_exec.shutdown(await_futures=False)


# ════════════════════════════════════════════════════════════════════════════════
# MOVEMENT LOGIC
# ════════════════════════════════════════════════════════════════════════════════

class GCodeExecutorNode(Node):
    """
    Executes a gcode print path on the UR5e via PILZ LIN Cartesian moves.

    Execution flow
    --------------
    1. PTP move to the first waypoint (gets the arm to the print start safely).
    2. Pre-plan all segments as concatenated PILZ LIN paths, chaining each
       trajectory into the next so the planner knows the exact joint state
       at every handoff.
    3. Ask for user confirmation before any motion begins.
    4. Execute each pre-planned trajectory in order.
    """

    def __init__(self):
        super().__init__('gcode_executor')

        self._plan_client = self.create_client(GetMotionPlan, PILZ_SERVICE)
        self._exec_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info('Waiting for MoveIt services…')
        self._plan_client.wait_for_service()
        self._exec_client.wait_for_server()
        self.get_logger().info('MoveIt services ready.')

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _get_final_robot_state(self, trajectory) -> RobotState:
        """Extract the ending RobotState from a trajectory for chained planning."""
        return _extract_end_state(trajectory)

    # ── PILZ LIN segment planning ─────────────────────────────────────────────

    def plan_lin_segment(self, poses: list, velocity_scale: float,
                         start_state: RobotState = None):
        """
        Plan a Cartesian segment through *poses* using sequential PILZ LIN moves.

        Parameters
        ----------
        poses          : ordered list of target Pose messages in base_link frame
        velocity_scale : fraction of MAX_SPEED_MM_PER_MIN  (0.0 – 1.0)
                         PILZ uses this as a true Cartesian TCP speed fraction.
        start_state    : optional RobotState for chained planning

        Returns
        -------
        RobotTrajectory on success, None on failure.
        """
        if not poses:
            return None

        trajs     = []
        cur_state = start_state

        for target_pose in poses:
            req    = _build_lin_request(self, target_pose, velocity_scale, cur_state)
            future = self._plan_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=PLANNING_TIMEOUT)

            if not future.done():
                self.get_logger().warn('PILZ LIN planning timed out.')
                return None

            resp = future.result()
            if resp.motion_plan_response.error_code.val != 1:
                self.get_logger().warn(
                    f'PILZ LIN failed: error code '
                    f'{resp.motion_plan_response.error_code.val}')
                return None

            traj = resp.motion_plan_response.trajectory
            trajs.append(traj)
            cur_state = _extract_end_state(traj)

        if len(trajs) == 1:
            return trajs[0]
        return _concatenate_trajectories(trajs)

    # ── Trajectory execution ──────────────────────────────────────────────────

    def execute_trajectory(self, trajectory, timeout_sec: float = 30.0) -> bool:
        """Execute a pre-planned RobotTrajectory.  Returns True on success."""
        pts = trajectory.joint_trajectory.points
        if pts:
            last_t = pts[-1].time_from_start
            traj_secs = last_t.sec + last_t.nanosec * 1e-9
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

    # ── Main execution flow ───────────────────────────────────────────────────

    def run(self, segments: list) -> None:
        """Execute all segments: approach → pre-plan → confirm → execute."""

        if not segments:
            self.get_logger().error('No segments to execute.')
            return

        first_pose = _make_pose(segments[0]['waypoints'][0])

        # ── Step 1: Plan approach ─────────────────────────────────────────────
        print(f"\n{'='*60}")
        print(f" Step 1: Planning approach to print start")
        print(f"  Target: ({first_pose.position.x:.4f}, "
              f"{first_pose.position.y:.4f}, "
              f"{first_pose.position.z:.4f})")
        print(f"{'='*60}")

        approach_traj = self.plan_lin_segment([first_pose], velocity_scale=0.2)
        if approach_traj is None:
            self.get_logger().error('PILZ LIN approach plan failed. Aborting.')
            return
        print('  Approach planned OK.')

        # ── Step 2: Pre-plan all segments ─────────────────────────────────────
        print(f"\n{'='*60}")
        print(f" Step 2: Pre-planning {len(segments)} segments…")
        print(f"{'='*60}")

        planned     = []
        start_state = self._get_final_robot_state(approach_traj) if approach_traj else None

        for i, seg in enumerate(segments):
            wps       = seg['waypoints']
            move_type = seg['move_type']
            vel_scale = seg['velocity_scale']

            # Segment 0: robot will arrive at wps[0] via approach, skip that point.
            path_wps = wps[1:] if i == 0 else wps

            if not path_wps:
                print(f"  Seg {i+1:>4}/{len(segments)}: {move_type:10s}  "
                      f"1 pt  — skip (already there)")
                planned.append(None)
                continue

            poses = [_make_pose(wp) for wp in path_wps]

            print(
                f"  Seg {i+1:>4}/{len(segments)}: {move_type:10s}  "
                f"{len(wps):4d} pts  "
                f"speed={seg['speed_mm_per_min']:6.0f} mm/min  "
                f"vel_scale={vel_scale:.3f}  "
                f"extrusion={seg['total_extrusion']:6.3f} mm  …",
                end='', flush=True,
            )

            traj = self.plan_lin_segment(poses, vel_scale, start_state)
            if traj is None:
                print(f"\n  ERROR: Planning failed for segment {i+1}. Aborting.")
                return

            print('  OK')
            planned.append(traj)
            start_state = self._get_final_robot_state(traj)

        # ── Pre-plan summary ──────────────────────────────────────────────────
        total_secs = 0.0
        for traj in planned:
            if traj is None:
                continue
            pts = traj.joint_trajectory.points
            if pts:
                t = pts[-1].time_from_start
                total_secs += t.sec + t.nanosec * 1e-9

        total_extrusion  = sum(s['total_extrusion'] for s in segments
                               if s['move_type'] == 'extrusion')
        extrusion_segs   = sum(1 for s in segments if s['move_type'] == 'extrusion')
        travel_segs      = sum(1 for s in segments if s['move_type'] == 'travel')
        h = int(total_secs // 3600)
        m = int((total_secs % 3600) // 60)
        s = int(total_secs % 60)

        print(f"\n{'='*60}")
        print(f" Pre-planning complete")
        print(f"  Extrusion segments : {extrusion_segs}")
        print(f"  Travel segments    : {travel_segs}")
        print(f"  Total extrusion    : {total_extrusion:.2f} mm")
        print(f"  Estimated time     : {h:02d}h {m:02d}m {s:02d}s")
        print(f"{'='*60}")

        # ── Connect to Klipper ────────────────────────────────────────────────
        klipper = KlipperComm()
        klipper_ready = klipper.connect()
        if not klipper_ready:
            print('  WARNING: Klipper not reachable — extrusion will be skipped.')

        # ── Confirm: move to first point ──────────────────────────────────────
        try:
            ans = input("\nPress ENTER to move to start position, or 'q' to quit: ")
        except EOFError:
            ans = ''
        if ans.strip().lower() == 'q':
            print('Aborted by user.')
            return

        print(f"\n{'='*60}")
        print(f" Step 3: Moving to start position")
        print(f"{'='*60}")

        if not self.execute_trajectory(approach_traj):
            self.get_logger().error('Failed to execute approach. Aborting.')
            return

        print('  At start position.')

        # ── Confirm: start print ──────────────────────────────────────────────
        try:
            ans = input("\nPress ENTER to start print, or 'q' to quit: ")
        except EOFError:
            ans = ''
        if ans.strip().lower() == 'q':
            print('Aborted by user.')
            return

        # ── Step 4: Execute all segments ──────────────────────────────────────
        print(f"\n{'='*60}")
        print(f" Step 4: Executing {len(planned)} segments")
        print(f"{'='*60}")

        ok_count     = 0
        skip_count   = 0
        cumulative_e = 0.0

        bg = _BackgroundPlanner()
        lookahead: concurrent.futures.Future | None = None

        def _submit_lookahead(seg_idx: int, end_state):
            """Submit a background PILZ LIN plan for segment seg_idx."""
            if seg_idx >= len(segments) or planned[seg_idx] is None:
                return None
            next_seg = segments[seg_idx]
            wps = next_seg['waypoints']
            if not wps:
                return None
            next_poses = [_make_pose(wp) for wp in wps]
            return bg.submit(next_poses, next_seg['velocity_scale'], end_state)

        for i, (seg, pretraj) in enumerate(zip(segments, planned)):
            move_type = seg['move_type']
            last_pose = _make_pose(seg['waypoints'][-1])

            print(f"\n  Seg {i+1:>4}/{len(segments)}: {move_type:10s}  "
                  f"→ ({last_pose.position.x:.4f}, "
                  f"{last_pose.position.y:.4f}, "
                  f"{last_pose.position.z:.4f})")

            if pretraj is None:
                print('    Already at target — skipping.')
                skip_count += 1
                lookahead = None
                continue

            wps      = seg['waypoints']
            path_wps = wps[1:] if i == 0 else wps
            if not path_wps:
                print('    No waypoints — skipping.')
                skip_count += 1
                lookahead = None
                continue
            poses = [_make_pose(wp) for wp in path_wps]

            # ── Obtain trajectory (lookahead or synchronous plan) ─────────────
            traj = None
            if lookahead is not None:
                try:
                    traj = lookahead.result(timeout=PLANNING_TIMEOUT)
                except Exception:
                    traj = None
                lookahead = None
                if traj is None:
                    print('    Lookahead missed — re-planning…', end='', flush=True)
                    traj = self.plan_lin_segment(
                        poses, seg['velocity_scale'], start_state=None)
                    print(' OK' if traj else ' FAILED')
            else:
                print('    Planning…', end='', flush=True)
                traj = self.plan_lin_segment(
                    poses, seg['velocity_scale'], start_state=None)
                print(' OK' if traj else ' FAILED')

            if traj is None:
                self.get_logger().error(f'PILZ LIN planning failed for segment {i+1}. Aborting.')
                bg.shutdown()
                return

            # ── Guard: skip zero-duration trajectories ────────────────────────
            pts    = traj.joint_trajectory.points
            last_t = pts[-1].time_from_start if pts else None
            traj_duration = (last_t.sec + last_t.nanosec * 1e-9) if last_t else 0.0
            if len(pts) < 2 or traj_duration < 1e-3:
                print(f'    Near-zero duration — already at target, skipping.')
                skip_count += 1
                continue

            # ── Submit lookahead for next segment while this one executes ─────
            end_state = self._get_final_robot_state(traj)
            result = _submit_lookahead(i + 1, end_state)
            if result:
                lookahead = result

            # ── Send extrusion command ────────────────────────────────────────
            if klipper_ready and move_type == 'extrusion' and seg['total_extrusion'] > 1e-9:
                seg_duration = traj_duration
                if seg_duration > 1e-9:
                    cumulative_e += seg['total_extrusion']
                    feedrate = (seg['total_extrusion'] / seg_duration) * 60.0
                    gcode_cmd = f'M82\nG1 E{cumulative_e:.4f} F{feedrate:.4f}'
                    print(f'    [Klipper] E{cumulative_e:.4f} F{feedrate:.2f} mm/min  ({seg_duration:.2f}s)')
                    threading.Thread(target=klipper.send_gcode, args=(gcode_cmd,), daemon=True).start()
                    if KLIPPER_DELAY_SEC > 0:
                        time.sleep(KLIPPER_DELAY_SEC)

            ok = self.execute_trajectory(traj)
            if not ok:
                lookahead = None
                print('    Execution failed — re-planning from current state…', end='', flush=True)
                traj = self.plan_lin_segment(
                    poses, seg['velocity_scale'], start_state=None)
                if traj is not None:
                    print(' OK')
                    ok = self.execute_trajectory(traj)
                if not ok:
                    self.get_logger().error(f'Execution failed for segment {i+1}. Aborting.')
                    bg.shutdown()
                    return

            ok_count += 1
            print('    OK')

        bg.shutdown()

        print(f"\n{'='*60}")
        print(f" Done.")
        print(f"  Segments executed : {ok_count}")
        print(f"  Segments skipped  : {skip_count}")
        print(f"{'='*60}")


# ──────────────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print('Usage: python3 gcode_executorPILZ.py <interpreted_gcode.csv>')
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.isfile(csv_path):
        print(f'ERROR: File not found: {csv_path}')
        sys.exit(1)

    print(f'Loading: {csv_path}')
    waypoints = load_csv(csv_path)
    print(f'  Loaded {len(waypoints)} waypoints')

    if len(waypoints) < 2:
        print('ERROR: Need at least 2 waypoints.')
        sys.exit(1)

    segments = group_by_segment(waypoints)
    extrusion_segs = sum(1 for s in segments if s['move_type'] == 'extrusion')
    travel_segs    = sum(1 for s in segments if s['move_type'] == 'travel')
    print(f'  Grouped into {len(segments)} segments '
          f'({extrusion_segs} extrusion, {travel_segs} travel)')

    rclpy.init()
    node = GCodeExecutorNode()
    node.run(segments)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
