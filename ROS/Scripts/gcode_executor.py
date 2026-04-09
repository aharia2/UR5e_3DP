#!/usr/bin/env python3
"""
gcode_executor.py
=================
Reads the CSV output of gcode_interpreter.py and executes the print path on
the UR5e via MoveIt's compute_cartesian_path.

Each segment (a group of consecutive same-type waypoints sharing a Segment_ID)
is planned as one continuous Cartesian path.  Toolhead speed is matched by
scaling the velocity: velocity_scale = target_speed / MAX_SPEED_MM_PER_MIN.

Usage
-----
  python3 gcode_executor.py <interpreted_gcode.csv>

Example
-------
  python3 gcode_executor.py "gcode_interpreted files/40x40x40.csv"
"""

import csv
import math
import os
import sys
import urllib.parse
import urllib.request
import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint, RobotState,
)
from shape_msgs.msg import SolidPrimitive

# ─── User settings ────────────────────────────────────────────────────────────

# Maximum TCP speed of the robot at velocity_scale = 1.0  (mm/min).
# velocity_scale = target_speed_mm_per_min / MAX_SPEED_MM_PER_MIN
# Adjust this to match the actual maximum configured speed of your MoveIt setup.
MAX_SPEED_MM_PER_MIN = 6000.0

# Acceleration scaling factor applied to all Cartesian moves (0.0 – 1.0).
# Set independently of velocity_scale. 1.0 = MoveIt's configured maximum.
ACCELERATION_SCALE = 0.5

# Cartesian path interpolation step (metres).  Smaller = smoother, slower to plan.
CARTESIAN_STEP = 0.001

# Jump threshold for GetCartesianPath.  4.0 is a safe default.
CARTESIAN_JUMP_THRESHOLD = 4.0

# Minimum fraction of the requested Cartesian path that must be computed
# before the result is accepted.
MIN_CARTESIAN_FRACTION = 0.95

# Timeout (seconds) for each planning service call.
PLANNING_TIMEOUT = 30.0

# ─── Klipper / Moonraker settings ─────────────────────────────────────────────

MOONRAKER_URL = "http://localhost:7125"

# Fixed delay (seconds) inserted at the start of the extrusion script so that
# Klipper's buffered commands stay in sync with the joint trajectory.
# Increase if extrusion starts too early; decrease if it starts too late.
KLIPPER_DELAY_SEC = 0.5

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

    For each extrusion waypoint j (j >= 1) in an extrusion segment, the G1 E
    command is stamped at the global time of waypoint j-1 — so extrusion starts
    when the robot leaves the previous waypoint and finishes on arrival at j.

    Travel segments produce no extrusion commands.

    Returns a list of (float, str) sorted by timestamp.
    """
    schedule = []
    cumulative_e = 0.0
    global_seg_start = 0.0  # seconds

    for seg, traj in zip(segments, planned):
        # Segment duration from the planned trajectory's final timestamp.
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
                ext_speed = wps[j]['extrusion_speed']  # mm/min
                # Stamp at waypoint j-1: extrusion starts here, ends at j.
                t_stamp = global_seg_start + wp_times[j - 1]
                schedule.append((t_stamp, f'G1 E{cumulative_e:.4f} F{ext_speed:.4f}'))
        else:
            # Travel — no extrusion; still track cumulative E (should be zero deltas).
            for wp in seg['waypoints']:
                cumulative_e += wp['extrusion_mm']

        global_seg_start += seg_duration

    return schedule


def schedule_to_gcode(schedule: list) -> str:
    """
    Convert a list of (timestamp_sec, gcode) pairs into a single G-code script
    using G4 P<ms> dwell commands to reproduce the timing.

    An initial G4 dwell of KLIPPER_DELAY_SEC is prepended to align Klipper's
    buffer execution with the start of robot motion.
    """
    lines = ['M82']  # absolute extrusion mode

    initial_delay_ms = max(0, int(KLIPPER_DELAY_SEC * 1000))
    if initial_delay_ms > 0:
        lines.append(f'G4 P{initial_delay_ms}')

    prev_t = 0.0
    for t, cmd in schedule:
        dt_ms = max(0, int((t - prev_t) * 1000))
        if dt_ms > 0:
            lines.append(f'G4 P{dt_ms}')
        lines.append(cmd)
        prev_t = t

    return '\n'.join(lines)


# ════════════════════════════════════════════════════════════════════════════════
# MOVEMENT LOGIC
# ════════════════════════════════════════════════════════════════════════════════

class GCodeExecutorNode(Node):
    """
    Executes a gcode print path on the UR5e via MoveIt Cartesian paths.

    Execution flow
    --------------
    1. PTP move to the first waypoint (gets the arm to the print start safely).
    2. Pre-plan all segments as Cartesian paths, chaining each trajectory into
       the next so the planner knows the exact joint state at every handoff.
    3. Ask for user confirmation before any motion begins.
    4. Execute each pre-planned trajectory in order.
    """

    def __init__(self):
        super().__init__('gcode_executor')

        self._cart_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')
        self._exec_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info('Waiting for MoveIt services…')
        self._cart_client.wait_for_service()
        self._exec_client.wait_for_server()
        self.get_logger().info('MoveIt services ready.')

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _get_final_robot_state(self, trajectory) -> RobotState:
        """Extract the ending RobotState from a trajectory for chained planning."""
        last_pt = trajectory.joint_trajectory.points[-1]
        rs = RobotState()
        rs.joint_state.name     = list(trajectory.joint_trajectory.joint_names)
        rs.joint_state.position = list(last_pt.positions)
        rs.joint_state.velocity = list(last_pt.velocities) if last_pt.velocities else []
        return rs

    # ── Cartesian path planning ───────────────────────────────────────────────

    def plan_cartesian_path(self, poses: list, velocity_scale: float,
                            start_state: RobotState = None):
        """
        Plan a continuous Cartesian path through *poses* at *velocity_scale*.

        Parameters
        ----------
        poses          : ordered list of target Pose messages in base_link frame
        velocity_scale : fraction of MAX_SPEED_MM_PER_MIN  (0.0 – 1.0)
        start_state    : optional RobotState for chained planning; when None
                         the current robot state is used

        Returns
        -------
        RobotTrajectory on success, None on failure.
        """
        req = GetCartesianPath.Request()
        req.header.stamp    = self.get_clock().now().to_msg()
        req.header.frame_id = 'base_link'
        req.group_name      = 'ur_arm'
        req.link_name       = 'tool_tip'
        req.waypoints       = poses
        req.max_step        = CARTESIAN_STEP
        req.jump_threshold  = CARTESIAN_JUMP_THRESHOLD
        req.avoid_collisions = True
        req.max_velocity_scaling_factor     = float(velocity_scale)
        req.max_acceleration_scaling_factor = float(ACCELERATION_SCALE)
        if start_state is not None:
            req.start_state = start_state

        future = self._cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=PLANNING_TIMEOUT)

        if not future.done():
            self.get_logger().warn('Cartesian path planning timed out.')
            return None

        response = future.result()
        if response.fraction < MIN_CARTESIAN_FRACTION:
            self.get_logger().warn(
                f'Cartesian path only {response.fraction*100:.1f}% complete '
                f'(minimum required: {MIN_CARTESIAN_FRACTION*100:.0f}%).'
            )
            return None

        return response.solution

    # ── PTP approach ──────────────────────────────────────────────────────────

    def move_ptp(self, target_pose: Pose) -> bool:
        """
        Point-to-point move to *target_pose* via the MoveGroup action.
        Used only for the initial approach to the print start position.
        """
        move_client = ActionClient(self, MoveGroup, '/move_action')
        move_client.wait_for_server()

        goal = MoveGroup.Goal()
        goal.request.group_name                      = 'ur_arm'
        goal.request.num_planning_attempts           = 30
        goal.request.allowed_planning_time           = 15.0
        goal.request.max_velocity_scaling_factor     = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        c  = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name       = 'tool_tip'
        sphere             = SolidPrimitive()
        sphere.type        = SolidPrimitive.SPHERE
        sphere.dimensions  = [0.01]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id           = 'base_link'
        oc.link_name                 = 'tool_tip'
        oc.orientation               = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.5
        oc.absolute_y_axis_tolerance = 0.5
        oc.absolute_z_axis_tolerance = 6.28
        oc.weight                    = 0.5
        c.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(c)

        future = move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done():
            self.get_logger().error('PTP: goal send timed out.')
            return False

        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('PTP: goal rejected.')
            return False

        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=30.0)
        if not rf.done():
            self.get_logger().error('PTP: result timed out.')
            return False

        if rf.result().result.error_code.val != 1:
            self.get_logger().error(
                f'PTP failed (error code {rf.result().result.error_code.val}).')
            return False

        return True

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

        # ── Step 1: Approach to print start ───────────────────────────────────
        print(f"\n{'='*60}")
        print(f" Step 1: Approach to print start (PTP)")
        print(f"  Target: ({first_pose.position.x:.4f}, "
              f"{first_pose.position.y:.4f}, "
              f"{first_pose.position.z:.4f})")
        print(f"{'='*60}")

        # Try Cartesian approach first; fall back to PTP for large joint motions.
        approach_traj = self.plan_cartesian_path([first_pose], velocity_scale=0.2)
        if approach_traj is None:
            print('  Cartesian approach failed — trying PTP.')
            if not self.move_ptp(first_pose):
                self.get_logger().error('Failed to reach start position. Aborting.')
                return
        else:
            if not self.execute_trajectory(approach_traj):
                self.get_logger().error('Failed to execute approach. Aborting.')
                return

        print('  Approach complete.')

        # ── Step 2: Pre-plan all segments ─────────────────────────────────────
        print(f"\n{'='*60}")
        print(f" Step 2: Pre-planning {len(segments)} segments…")
        print(f"{'='*60}")

        planned     = []   # pre-planned trajectories (None = skip, already there)
        start_state = None

        for i, seg in enumerate(segments):
            wps       = seg['waypoints']
            move_type = seg['move_type']
            vel_scale = seg['velocity_scale']

            # For the first segment the robot just arrived at wps[0] via the
            # approach move, so skip it.  For all later segments the robot ends
            # the previous segment at a different position, so include all points.
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

            traj = self.plan_cartesian_path(poses, vel_scale, start_state)
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

        # ── Build extrusion schedule ──────────────────────────────────────────
        print(f"\n{'='*60}")
        print(f" Building extrusion schedule…")
        schedule = build_extrusion_schedule(segments, planned)
        gcode_script = schedule_to_gcode(schedule)
        print(f"  {len(schedule)} extrusion commands scheduled")
        print(f"{'='*60}")

        # ── Connect to Klipper ────────────────────────────────────────────────
        klipper = KlipperComm()
        klipper_ready = klipper.connect()
        if not klipper_ready:
            print('  WARNING: Klipper not reachable — extrusion will be skipped.')

        # ── User confirmation ─────────────────────────────────────────────────
        try:
            ans = input("\nPress ENTER to begin execution, or 'q' to quit: ")
        except EOFError:
            ans = ''
        if ans.strip().lower() == 'q':
            print('Aborted by user.')
            return

        # ── Step 3: Send extrusion script then execute ────────────────────────
        print(f"\n{'='*60}")
        print(f" Step 3: Executing {len(planned)} segments")
        print(f"{'='*60}")

        if klipper_ready and schedule:
            print('  Sending extrusion script to Klipper…', end='', flush=True)
            if klipper.send_gcode(gcode_script):
                print('  OK')
            else:
                print('  FAILED — continuing without extrusion.')
                klipper_ready = False

        ok_count   = 0
        skip_count = 0

        for i, (seg, traj) in enumerate(zip(segments, planned)):
            move_type = seg['move_type']
            last_pose = _make_pose(seg['waypoints'][-1])

            print(f"\n  Seg {i+1:>4}/{len(segments)}: {move_type:10s}  "
                  f"→ ({last_pose.position.x:.4f}, "
                  f"{last_pose.position.y:.4f}, "
                  f"{last_pose.position.z:.4f})")

            if traj is None:
                print('    Already at target — skipping.')
                skip_count += 1
                continue

            ok = self.execute_trajectory(traj)
            if not ok:
                print('    Cartesian execution failed — trying PTP fallback.')
                ok = self.move_ptp(last_pose)
                if not ok:
                    print(f'    WARNING: PTP fallback also failed — skipping segment {i+1}.')
                    skip_count += 1
                    continue

            ok_count += 1
            print('    OK')

        print(f"\n{'='*60}")
        print(f" Done.")
        print(f"  Segments executed : {ok_count}")
        print(f"  Segments skipped  : {skip_count}")
        print(f"{'='*60}")


# ──────────────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print('Usage: python3 gcode_executor.py <interpreted_gcode.csv>')
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
