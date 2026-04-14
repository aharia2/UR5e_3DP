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
import json
import os
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState

# ─── User settings ────────────────────────────────────────────────────────────

# Maximum TCP speed of the robot at velocity_scale = 1.0  (mm/min).
# velocity_scale = target_speed_mm_per_min / MAX_SPEED_MM_PER_MIN
MAX_SPEED_MM_PER_MIN = 60000.0

# Speed for the initial approach move to the print start position (mm/min).
APPROACH_SPEED_MM_PER_MIN = 1000.0

# Acceleration scaling factor applied to all Cartesian moves (0.0 – 1.0).
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

# Delay (seconds) between sending the Klipper extrusion command and starting
# robot motion.  Increase if extrusion starts too late; decrease if too early.
KLIPPER_DELAY_SEC = 0.45

# ──────────────────────────────────────────────────────────────────────────────


# ─── CSV loading ──────────────────────────────────────────────────────────────

def load_csv(path: str) -> list:
    waypoints = []
    with open(path, 'r') as fh:
        for raw_row in csv.reader(fh):
            row = [c.strip() for c in raw_row]
            if not row or row[0].startswith('#'):
                continue
            if row[0].upper() == 'X':
                continue
            if len(row) < 13:
                continue
            try:
                waypoints.append({
                    'x':               float(row[0]) * 0.001,  # mm → m
                    'y':               float(row[1]) * 0.001,
                    'z':               float(row[2]) * 0.001,
                    'qx':              float(row[3]),
                    'qy':              float(row[4]),
                    'qz':              float(row[5]),
                    'qw':              float(row[6]),
                    'speed':           float(row[7]),
                    'extrusion_mm':    float(row[8]),
                    'extrusion_speed': float(row[9]),
                    'move_type':       row[10],
                    'segment_id':      int(float(row[11])),
                    'gcode_line':      int(float(row[12])),
                })
            except (ValueError, IndexError):
                pass
    return waypoints


def group_by_segment(waypoints: list) -> list:
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
        'segment_id':       seg_id,
        'move_type':        wps[0]['move_type'],
        'waypoints':        wps,
        'speed_mm_per_min': speed,
        'velocity_scale':   min(1.0, max(0.01, speed / MAX_SPEED_MM_PER_MIN)),
        'total_extrusion':  sum(wp['extrusion_mm'] for wp in wps),
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


# ─── Klipper communication ────────────────────────────────────────────────────

class KlipperComm:
    """Send G-code to Klipper via the Moonraker HTTP API."""

    def __init__(self, url: str = MOONRAKER_URL):
        self.url = url.rstrip('/')

    def connect(self) -> bool:
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


# ════════════════════════════════════════════════════════════════════════════════
# MOVEMENT LOGIC
# ════════════════════════════════════════════════════════════════════════════════

class GCodeExecutorNode(Node):

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

    def _get_final_robot_state(self, trajectory) -> RobotState:
        last_pt = trajectory.joint_trajectory.points[-1]
        rs = RobotState()
        rs.joint_state.name     = list(trajectory.joint_trajectory.joint_names)
        rs.joint_state.position = list(last_pt.positions)
        rs.joint_state.velocity = list(last_pt.velocities) if last_pt.velocities else []
        return rs

    def plan_cartesian_path(self, poses: list, velocity_scale: float,
                            start_state: RobotState = None):
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

    def execute_trajectory(self, trajectory, timeout_sec: float = 30.0) -> bool:
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

    def run(self, segments: list) -> None:

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

        approach_vel = APPROACH_SPEED_MM_PER_MIN / MAX_SPEED_MM_PER_MIN
        approach_traj = self.plan_cartesian_path([first_pose], velocity_scale=approach_vel)
        if approach_traj is None:
            self.get_logger().error('Cartesian approach plan failed. Aborting.')
            return
        print('  Approach planned OK.')

        # ── Step 2: Pre-plan all segments ─────────────────────────────────────
        print(f"\n{'='*60}")
        print(f" Step 2: Pre-planning {len(segments)} segments…")
        print(f"{'='*60}")

        planned     = []
        start_state = self._get_final_robot_state(approach_traj)

        for i, seg in enumerate(segments):
            wps       = seg['waypoints']
            move_type = seg['move_type']
            vel_scale = seg['velocity_scale']

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

        total_extrusion = sum(s['total_extrusion'] for s in segments
                              if s['move_type'] == 'extrusion')
        extrusion_segs  = sum(1 for s in segments if s['move_type'] == 'extrusion')
        travel_segs     = sum(1 for s in segments if s['move_type'] == 'travel')
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

        # ── Confirm: move to start position ───────────────────────────────────
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

            # ── Send Klipper extrusion command ────────────────────────────────
            if klipper_ready and move_type == 'extrusion' and seg['total_extrusion'] > 1e-9:
                pts = traj.joint_trajectory.points
                seg_duration = 0.0
                if pts:
                    t = pts[-1].time_from_start
                    seg_duration = t.sec + t.nanosec * 1e-9
                if seg_duration > 1e-9:
                    cumulative_e += seg['total_extrusion']
                    feedrate = (seg['total_extrusion'] / seg_duration) * 60.0
                    gcode_cmd = f'M82\nG1 E{cumulative_e:.4f} F{feedrate:.4f}'
                    print(f'    [Klipper] E{cumulative_e:.4f} F{feedrate:.2f} mm/min  ({seg_duration:.2f}s)')
                    threading.Thread(
                        target=klipper.send_gcode, args=(gcode_cmd,), daemon=True).start()
                    if KLIPPER_DELAY_SEC > 0:
                        time.sleep(KLIPPER_DELAY_SEC)

            ok = self.execute_trajectory(traj)
            if not ok:
                print(f'    WARNING: Execution failed — skipping segment {i+1}.')
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
