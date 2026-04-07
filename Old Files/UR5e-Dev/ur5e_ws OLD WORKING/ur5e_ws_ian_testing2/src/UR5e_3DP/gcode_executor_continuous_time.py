#!/usr/bin/env python3
"""
gcode_executor_continuous_time.py

Moves the UR5e through waypoints as continuous Cartesian paths.
Distributes extrusion across the trajectory using per-interval timed commands
matched to the robot's velocity profile, to reduce over-extrusion at corners.

Key difference from gcode_executor_continuous_NEW.py
-----------------------------------------------------
Instead of one extrusion command per segment, the planned trajectory is
time-parameterized and split into intervals. Each interval gets an extrusion
command proportional to the joint velocity (proxy for TCP speed) in that
interval, fired by a threading.Timer at the correct wall-clock offset.

This means Klipper extrudes more during fast straight sections and less during
slow corner sections, matching the robot's actual velocity profile.

KLIPPER_START_LATENCY:
  Negative (e.g. -0.20): Klipper is slower — send first command early,
                          sleep |latency| before starting the robot.
  Positive              : Klipper is faster — delay first command via timer.

Usage:
    python3 gcode_executor_continuous_time.py                        # default waypoints_continuous.csv
    python3 gcode_executor_continuous_time.py /path/to/file.csv
    python3 gcode_executor_continuous_time.py "/path/to/print.gcode"
"""

import rclpy
import csv
import os
import sys
import math
import time as _time
import threading
import importlib.util

# Import conversion helpers from gcode_to_waypoints_continuous (same directory)
_gtw_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'gcode_to_waypoints_continuous.py')
_spec = importlib.util.spec_from_file_location('gcode_to_waypoints_continuous', _gtw_path)
_gcode_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_gcode_mod)

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (Constraints, PositionConstraint,
                             OrientationConstraint)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import tf2_ros
from Klipper_communcation import KlipperComm

# ─── Tunable parameters ────────────────────────────────────────────────────
EXTRUDE_PER_POINT    = 1.0    # mm per waypoint when CSV has no extrude column
DEFAULT_VELOCITY     = 0.03    # default velocity scaling (0.0–1.0)
POSITION_TOLERANCE   = 0.008  # 8 mm — acceptable TCP error before triggering extrusion
CARTESIAN_STEP       = 0.005  # 5 mm interpolation step along each segment
KLIPPER_FEEDRATE     = 600    # mm/min fallback feedrate if schedule cannot be built

# ─── Start-latency compensation ────────────────────────────────────────────
# Positive value means Klipper starts extruding faster than the robot starts
# moving (common case — delay the first Klipper command by this many seconds).
# Negative value means the robot moves before Klipper starts extruding.
# Measure empirically: run a single short segment and observe whether
# extrusion leads or lags motion at the start, then adjust.
KLIPPER_START_LATENCY = -0.2  # seconds  (negative = send Klipper command before robot moves)
# ───────────────────────────────────────────────────────────────────────────


def _run_extrusion_sequence(schedule, extrude_pub, klipper):
    """
    Background thread: sends the entire extrusion sequence to Klipper in
    ONE HTTP request so all moves are queued atomically before the robot starts.
    Klipper's move buffer then executes them at the given feedrates.
    """
    t0 = _time.monotonic()

    # Publish each command to ROS topic for visualization/logging
    for e_i, fr_i in schedule:
        msg = Float64()
        msg.data = e_i
        extrude_pub.publish(msg)

    # Send entire sequence to Klipper as one HTTP round-trip
    if klipper is not None:
        t_send = _time.monotonic()
        klipper.send_extrusion_sequence(schedule)
        send_took = _time.monotonic() - t_send
        total_e = sum(e for e, _ in schedule)
        print(f"  [thread] {len(schedule)} cmds queued in {send_took:.3f}s  "
              f"total={total_e:.4f} mm  elapsed={_time.monotonic()-t0:.3f}s")


def _build_extrusion_schedule(traj, total_extrude):
    """
    Split total_extrude equally across the planned trajectory's time intervals.

    Each trajectory interval from GetCartesianPath represents approximately
    CARTESIAN_STEP (5 mm) of TCP travel, so equal extrusion per interval gives
    constant filament density along the path regardless of robot speed.
    Feedrate for each interval is e_i/dt so Klipper automatically matches the
    robot's actual speed — fast feedrate on straights, slow through corners.

    Returns list of (t_offset_sec, e_mm, feedrate_mm_min).
    """
    pts = traj.joint_trajectory.points
    if len(pts) < 2:
        return []

    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts]

    intervals = []
    for i in range(1, len(pts)):
        dt = times[i] - times[i - 1]
        if dt < 1e-6:
            continue
        intervals.append((times[i - 1], dt))

    if not intervals:
        return []

    e_per_interval = total_extrude / len(intervals)

    schedule = []
    for t_start, dt in intervals:
        schedule.append((t_start, e_per_interval, (e_per_interval / dt) * 60.0))

    return schedule


def make_pose(x, y, z, qx, qy, qz, qw):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


# ── Waypoint loading ───────────────────────────────────────────────────────

def load_waypoints(csv_file):
    """
    Load poses, extrusion amounts, velocity scales, and is_travel flags from CSV.
    CSV format: x, y, z, qx, qy, qz, qw [, extrude_mm [, velocity_scale [, is_travel]]]
    """
    poses      = []
    extrudes   = []
    velocities = []
    is_travels = []

    with open(csv_file, 'r') as f:
        for row in csv.reader(f):
            row = [c.strip() for c in row if c.strip()]
            if len(row) < 7:
                continue
            try:
                x, y, z, qx, qy, qz, qw = [float(v) for v in row[:7]]
                e  = float(row[7]) if len(row) >= 8 else EXTRUDE_PER_POINT
                v  = float(row[8]) if len(row) >= 9 else DEFAULT_VELOCITY
                it = bool(int(float(row[9]))) if len(row) >= 10 else False
                poses.append(make_pose(x, y, z, qx, qy, qz, qw))
                extrudes.append(e)
                velocities.append(max(0.01, min(1.0, v)))
                is_travels.append(it)
            except ValueError:
                pass  # skip header / comment lines

    return poses, extrudes, velocities, is_travels


def group_segments(poses, extrudes, velocities, is_travels):
    """
    Group consecutive waypoints of the same type into segments.

    Returns a list of dicts:
        {
          'type':       'extrude' | 'travel',
          'poses':      [Pose, ...],
          'extrudes':   [float, ...],   # per-waypoint extrusion amounts
          'velocities': [float, ...],
          'total_extrude': float,       # sum of extrudes for the segment
          'velocity':   float,          # average velocity for the segment
        }
    """
    if not poses:
        return []

    segments = []
    cur_type  = 'travel' if is_travels[0] else 'extrude'
    cur_seg   = {'type': cur_type, 'poses': [], 'extrudes': [], 'velocities': []}

    for pose, extrude, vel, is_travel in zip(poses, extrudes, velocities, is_travels):
        gtype = 'travel' if is_travel else 'extrude'
        if gtype != cur_type:
            _finalise_segment(cur_seg)
            segments.append(cur_seg)
            cur_type = gtype
            cur_seg  = {'type': cur_type, 'poses': [], 'extrudes': [], 'velocities': []}
        cur_seg['poses'].append(pose)
        cur_seg['extrudes'].append(extrude)
        cur_seg['velocities'].append(vel)

    _finalise_segment(cur_seg)
    segments.append(cur_seg)
    return segments


def _finalise_segment(seg):
    seg['total_extrude'] = sum(seg['extrudes'])
    seg['velocity']      = (sum(seg['velocities']) / len(seg['velocities'])
                            if seg['velocities'] else DEFAULT_VELOCITY)


# ── ROS node ───────────────────────────────────────────────────────────────

class GCodeExecutorContinuous(Node):
    def __init__(self):
        super().__init__('gcode_executor_continuous')

        self.cart_client    = self.create_client(GetCartesianPath,
                                                  '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory,
                                           '/execute_trajectory')
        self.extrude_pub    = self.create_publisher(Float64,
                                                    '/extruder_command', 10)
        self.marker_pub     = self.create_publisher(Marker,
                                                    '/visualization_marker', 10)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        print("Waiting for MoveIt services...")
        self.cart_client.wait_for_service()
        self.execute_client.wait_for_server()
        print("Ready!")

    # ── TF helpers ────────────────────────────────────────────────────────

    def get_tcp_position(self):
        """Return current (x, y, z) of tool_tip in base_link, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'tool_tip', rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def distance_to_target(self, target_pose):
        """Euclidean distance from current TCP to target pose (metres)."""
        pos = self.get_tcp_position()
        if pos is None:
            return float('inf')
        dx = pos[0] - target_pose.position.x
        dy = pos[1] - target_pose.position.y
        dz = pos[2] - target_pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    # ── Motion helpers ────────────────────────────────────────────────────

    def move_ptp(self, target_pose):
        """PTP move to a single target pose."""
        move_client = ActionClient(self, MoveGroup, '/move_action')
        move_client.wait_for_server()

        goal = MoveGroup.Goal()
        goal.request.group_name = "ur_arm"
        goal.request.num_planning_attempts = 30
        goal.request.allowed_planning_time  = 15.0
        goal.request.max_velocity_scaling_factor     = 0.03
        goal.request.max_acceleration_scaling_factor = 0.03

        c  = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name       = "tool_tip"
        sphere             = SolidPrimitive()
        sphere.type        = SolidPrimitive.SPHERE
        sphere.dimensions  = [0.01]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name       = "tool_tip"
        oc.orientation     = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.5
        oc.absolute_y_axis_tolerance = 0.5
        oc.absolute_z_axis_tolerance = 6.28
        oc.weight = 0.5
        c.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(c)

        future = move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done():
            print("ERROR: PTP goal send timed out")
            return False
        gh = future.result()
        if not gh.accepted:
            print("ERROR: PTP goal rejected")
            return False

        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=30.0)
        if not rf.done():
            print("ERROR: PTP result timed out")
            return False
        result = rf.result()
        if result.result.error_code.val != 1:
            print(f"ERROR: PTP failed (code {result.result.error_code.val})")
            return False

        print("SUCCESS: PTP move complete")
        return True

    def get_final_robot_state(self, trajectory):
        """Extract the final RobotState from a planned trajectory."""
        from moveit_msgs.msg import RobotState
        last_pt = trajectory.joint_trajectory.points[-1]
        rs = RobotState()
        rs.joint_state.name     = list(trajectory.joint_trajectory.joint_names)
        rs.joint_state.position = list(last_pt.positions)
        rs.joint_state.velocity = list(last_pt.velocities) if last_pt.velocities else []
        return rs

    def plan_cartesian_path(self, waypoint_poses, velocity_scale=DEFAULT_VELOCITY,
                            start_state=None):
        """
        Plan a Cartesian path through one or more waypoints.

        Passing multiple poses in waypoint_poses produces a single continuous
        Cartesian trajectory rather than a sequence of stop-and-go moves.

        Returns the RobotTrajectory, or None on failure.
        """
        req = GetCartesianPath.Request()
        req.header.stamp    = self.get_clock().now().to_msg()
        req.header.frame_id = "base_link"
        req.group_name      = "ur_arm"
        req.link_name       = "tool_tip"
        req.waypoints       = waypoint_poses   # list of Pose
        req.max_step        = CARTESIAN_STEP
        req.jump_threshold  = 4.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor     = float(velocity_scale)
        req.max_acceleration_scaling_factor = float(velocity_scale)
        if start_state is not None:
            req.start_state = start_state

        future = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            print("  WARNING: CartesianPath planning timed out (30s)")
            return None
        response = future.result()

        if response.fraction < 0.95:
            print(f"  WARNING: Path only {response.fraction*100:.1f}% computed")
            return None
        return response.solution

    def execute_trajectory(self, trajectory, timeout_sec=30.0):
        """Execute a pre-planned RobotTrajectory. Returns True on success."""
        pts = trajectory.joint_trajectory.points
        if pts:
            last_t = pts[-1].time_from_start
            traj_secs = last_t.sec + last_t.nanosec * 1e-9
            timeout_sec = max(timeout_sec, traj_secs * 3.0 + 5.0)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            print("  ERROR: Execution goal send timed out")
            return False
        gh = send_future.result()
        if not gh.accepted:
            print("  ERROR: Execution rejected")
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            print(f"  ERROR: Execution timed out after {timeout_sec:.0f}s")
            return False
        return True

    # ── Extrusion ─────────────────────────────────────────────────────────

    def start_extrusion(self, total_mm, feedrate_mm_min, klipper=None):
        """
        Send a single extrusion command (fallback / travel-free segments).
        """
        msg = Float64()
        msg.data = total_mm
        self.extrude_pub.publish(msg)

        if klipper is not None:
            klipper.send_extrusion(total_mm, feedrate=feedrate_mm_min)

        print(f"  → EXTRUDE {total_mm:.3f} mm @ {feedrate_mm_min:.1f} mm/min (single command)")

    def execute_segment_synchronized(self, seg, traj, klipper):
        """
        Execute a Cartesian trajectory with a per-segment threaded extrusion sequence.

        For each segment:
          1. Build per-interval schedule from joint-velocity magnitudes.
             Corners get less extrusion (low velocity) than straight sections.
          2. Shift every t_traj by KLIPPER_START_LATENCY:
               adjusted_t = t_traj + KLIPPER_START_LATENCY
             Negative latency → commands fire before the robot starts.
          3. Launch a background thread that wakes at each command's wall-clock
             time (t_robot_start + adjusted_t) and fires it to Klipper.
          4. Sleep abs(KLIPPER_START_LATENCY) on the main thread (lead-in),
             record t_robot_start, then call execute_trajectory.
          5. Join the thread after the trajectory finishes.

        Falls back to a single timed command if joint velocities are unavailable.
        """
        total_extrude = seg['total_extrude']

        pts = traj.joint_trajectory.points
        traj_secs = 0.0
        if pts:
            last_t = pts[-1].time_from_start
            traj_secs = last_t.sec + last_t.nanosec * 1e-9

        lead     = abs(KLIPPER_START_LATENCY)
        schedule = _build_extrusion_schedule(traj, total_extrude)

        # ── Fallback: single command ──────────────────────────────────────────
        if not schedule:
            duration = traj_secs + lead
            feedrate = (total_extrude / duration) * 60.0 if duration > 0 else KLIPPER_FEEDRATE
            print(f"  [time] No velocity data — single command "
                  f"{total_extrude:.4f} mm @ {feedrate:.1f} mm/min  lead={lead:.3f}s")
            self.start_extrusion(total_extrude, feedrate, klipper)
            _time.sleep(lead)
            return self.execute_trajectory(traj)

        # ── Scale feedrates so total queue duration = traj_secs + lead ──────────
        # Each interval originally takes dt = e_i / fr_i * 60 seconds.
        # Dividing all feedrates by time_scale stretches the total to traj_secs+lead.
        # This means Klipper starts lead seconds before the robot and finishes
        # exactly when the robot finishes.
        # The velocity profile shape is preserved — corners still get less
        # extrusion proportionally.
        time_scale = (traj_secs + lead) / traj_secs if traj_secs > 0 else 1.0
        scaled = [(e_i, fr_i / time_scale) for (_, e_i, fr_i) in schedule]

        print(f"  [time] {len(scaled)} commands  total={total_extrude:.4f} mm  "
              f"traj={traj_secs:.2f}s  lead={lead:.3f}s  scale={time_scale:.3f}")

        # ── Thread sends all commands upfront, Klipper buffers them ───────────
        # No per-command sleep — Klipper's move buffer executes them at the
        # right feedrates.  Thread runs concurrently with the main thread's sleep.
        thread = threading.Thread(
            target=_run_extrusion_sequence,
            args=(scaled, self.extrude_pub, klipper),
            daemon=True)
        thread.start()

        # ── Sleep lead then start robot ───────────────────────────────────────
        # By the time execute_trajectory is called, Klipper has been filling
        # its buffer for lead seconds and will finish exactly when the robot does.
        _time.sleep(lead)
        ok = self.execute_trajectory(traj)

        thread.join(timeout=traj_secs + lead + 5.0)
        if thread.is_alive():
            print("  [time] WARNING: extrusion thread did not finish in time.")

        return ok

    def verify_position(self, target_pose, label=""):
        """Log TCP position and return distance to target."""
        rclpy.spin_once(self, timeout_sec=0.1)
        err = self.distance_to_target(target_pose)
        pos = self.get_tcp_position()
        if pos:
            print(f"  TCP{' '+label if label else ''}: "
                  f"({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})  "
                  f"error: {err*1000:.1f} mm")
        return err

    # ── Visualization ─────────────────────────────────────────────────────

    def publish_path_marker(self, all_segments, current_seg_idx):
        """Visualize remaining extrusion path (blue) and travel moves (grey) in RViz."""
        for color, seg_type, r, g, b in [
            ('extrude', 'extrude', 0.0, 0.4, 1.0),
            ('travel',  'travel',  0.5, 0.5, 0.5),
        ]:
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp    = self.get_clock().now().to_msg()
            marker.ns              = f"gcode_{color}"
            marker.id              = 0 if color == 'extrude' else 1
            marker.type            = Marker.LINE_STRIP
            marker.action          = Marker.ADD
            marker.scale.x         = 0.002
            marker.color           = ColorRGBA(r=r, g=g, b=b, a=0.8)
            from geometry_msgs.msg import Point
            for i, seg in enumerate(all_segments):
                if i < current_seg_idx or seg['type'] != seg_type:
                    continue
                for p in seg['poses']:
                    pt = Point()
                    pt.x = p.position.x
                    pt.y = p.position.y
                    pt.z = p.position.z
                    marker.points.append(pt)
            self.marker_pub.publish(marker)


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = GCodeExecutorContinuous()

    klipper = KlipperComm()

    # ── Load waypoints ──────────────────────────────────────────────────
    default_csv = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'waypoints_continuous.csv')
    input_file = sys.argv[1] if len(sys.argv) > 1 else default_csv

    if not os.path.isfile(input_file):
        print(f"ERROR: File not found: {input_file}")
        rclpy.shutdown()
        return

    # ── Auto-detect G-code vs CSV ──────────────────────────────────────
    if input_file.lower().endswith('.gcode'):
        print(f"G-code input detected — converting on-the-fly: {input_file}")
        raw      = _gcode_mod.parse_gcode(input_file)
        filtered = _gcode_mod.downsample(raw, _gcode_mod.MIN_SEGMENT_M)
        extrusion_n = sum(1 for p in filtered if not p['is_travel'])
        travel_n    = sum(1 for p in filtered if p['is_travel'])
        print(f"  {len(raw)} raw moves → {len(filtered)} waypoints "
              f"(extrusion: {extrusion_n}, travel: {travel_n})")

        if _gcode_mod.USE_BED_LEVELING:
            _, _, (bqx, bqy, bqz, bqw) = _gcode_mod._get_bed_transform()
        else:
            bqx, bqy, bqz, bqw = _gcode_mod.QX, _gcode_mod.QY, _gcode_mod.QZ, _gcode_mod.QW

        poses      = []
        extrudes   = []
        velocities = []
        is_travels = []
        for p in filtered:
            rx, ry, rz = _gcode_mod.gcode_to_robot(p['x'], p['y'], p['z'])
            poses.append(make_pose(rx, ry, rz, bqx, bqy, bqz, bqw))
            e = p['e_delta'] if _gcode_mod.EXTRUDE_MM == 'auto' else float(_gcode_mod.EXTRUDE_MM)
            extrudes.append(e)
            velocities.append(_gcode_mod.VELOCITY_SCALE)
            is_travels.append(p['is_travel'])
    else:
        poses, extrudes, velocities, is_travels = load_waypoints(input_file)

    if len(poses) < 2:
        print("ERROR: Need at least 2 waypoints")
        rclpy.shutdown()
        return

    extrusion_count = sum(1 for t in is_travels if not t)
    travel_count    = sum(1 for t in is_travels if t)
    print(f"Loaded {len(poses)} waypoints — extrusion: {extrusion_count}, travel: {travel_count}")

    # ── Group into continuous segments ──────────────────────────────────
    segments = group_segments(poses, extrudes, velocities, is_travels)
    extrude_segs = sum(1 for s in segments if s['type'] == 'extrude')
    travel_segs  = sum(1 for s in segments if s['type'] == 'travel')
    print(f"Grouped into {len(segments)} segments "
          f"({extrude_segs} extrusion, {travel_segs} travel)")

    # ── Step 1: Move to first waypoint ──────────────────────────────────
    first_pose = segments[0]['poses'][0]
    print(f"\n{'='*60}")
    print(f"Step 1: Approach to first waypoint")
    print(f"  Target: ({first_pose.position.x:.4f}, "
                       f"{first_pose.position.y:.4f}, "
                       f"{first_pose.position.z:.4f})")
    print(f"{'='*60}")

    traj0 = node.plan_cartesian_path([first_pose], velocities[0])
    if traj0 is None:
        print("  Cartesian approach failed — falling back to PTP.")
        if not node.move_ptp(first_pose):
            print("Failed to reach start position.")
            klipper.close()
            rclpy.shutdown()
            return
    else:
        if not node.execute_trajectory(traj0):
            print("Failed to execute approach trajectory.")
            klipper.close()
            rclpy.shutdown()
            return

    # ── Step 2: Pre-plan all segments ────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"Step 2: Pre-planning {len(segments)} segments...")
    print(f"{'='*60}")

    planned_segments = []
    start_state = None

    for i, seg in enumerate(segments):
        seg_type  = seg['type']
        n_poses   = len(seg['poses'])
        vel       = seg['velocity']

        # For single-pose segments (first point of a new group), we're already
        # there — nothing to plan. Skip and chain from current state.
        if n_poses == 1:
            print(f"  Segment {i+1}/{len(segments)}: {seg_type}  1 pose — skipping (already there)")
            planned_segments.append(None)
            continue

        label = (f"{seg_type}  {n_poses} poses  "
                 f"vel={vel*100:.0f}%  "
                 f"extrude={seg['total_extrude']:.3f} mm" if seg_type == 'extrude'
                 else f"{seg_type}  {n_poses} poses  vel={vel*100:.0f}%")

        print(f"  Segment {i+1}/{len(segments)}: {label}  ...", end="", flush=True)

        # Plan through all poses in the segment as one continuous path
        # Skip the first pose — the robot is already there from the previous segment
        path_poses = seg['poses'][1:] if i > 0 else seg['poses']
        if not path_poses:
            print(" skip (no new poses)")
            planned_segments.append(None)
            continue

        traj = node.plan_cartesian_path(path_poses, vel, start_state)
        if traj is None:
            print(f"\n  ERROR: Could not plan segment {i+1}, aborting.")
            klipper.close()
            rclpy.shutdown()
            return

        print(" OK")
        planned_segments.append(traj)
        start_state = node.get_final_robot_state(traj)

    # ── Estimate total execution time ────────────────────────────────────
    total_traj_secs = 0.0
    for traj in planned_segments:
        if traj is None:
            continue
        pts = traj.joint_trajectory.points
        if pts:
            last_t = pts[-1].time_from_start
            total_traj_secs += last_t.sec + last_t.nanosec * 1e-9

    total_extrude_mm = sum(s['total_extrude'] for s in segments if s['type'] == 'extrude')
    hours   = int(total_traj_secs // 3600)
    minutes = int((total_traj_secs % 3600) // 60)
    seconds = int(total_traj_secs % 60)

    print(f"\n{'='*60}")
    print(f"Planning complete — {len(planned_segments)} segments ready")
    print(f"  Extrusion segments : {extrude_segs}")
    print(f"  Travel segments    : {travel_segs}")
    print(f"  Total extrusion    : {total_extrude_mm:.2f} mm")
    print(f"  Estimated time     : {hours:02d}h {minutes:02d}m {seconds:02d}s")
    print(f"{'='*60}")

    node.publish_path_marker(segments, 0)

    # ── Wait for user confirmation ────────────────────────────────────────
    try:
        ans = input("\nPress ENTER to start execution, or type 'q' + ENTER to quit: ")
    except EOFError:
        ans = ""
    if ans.strip().lower() == 'q':
        print("Aborted by user.")
        klipper.close()
        rclpy.shutdown()
        return

    # ── Step 3: Execute all segments ─────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"Step 3: Executing {len(planned_segments)} segments")
    print(f"{'='*60}")

    extrude_success = 0
    travel_success  = 0
    skip_count      = 0

    for i, (seg, traj) in enumerate(zip(segments, planned_segments)):
        seg_type  = seg['type']
        last_pose = seg['poses'][-1]
        vel       = seg['velocity']
        n_poses   = len(seg['poses'])

        print(f"\nSegment {i+1}/{len(segments)}: {seg_type}  "
              f"({n_poses} poses  →  "
              f"({last_pose.position.x:.4f}, "
              f"{last_pose.position.y:.4f}, "
              f"{last_pose.position.z:.4f}))")

        if traj is None:
            print("  Already at target — skipping.")
            continue

        node.publish_path_marker(segments, i)

        if seg_type == 'extrude':
            ok = node.execute_segment_synchronized(seg, traj, klipper)
        else:
            ok = node.execute_trajectory(traj)
        if not ok:
            print(f"  → Cartesian failed — falling back to PTP to segment end.")
            ok = node.move_ptp(last_pose)
            if not ok:
                print(f"  WARNING: PTP fallback also failed — skipping segment {i+1}.")
                skip_count += 1
                continue

        if seg_type == 'extrude':
            err = node.verify_position(last_pose, "end of extrusion segment")
            if err <= POSITION_TOLERANCE:
                extrude_success += 1
            else:
                print(f"  WARNING: Position error {err*1000:.1f} mm — "
                      f"extrusion may not align with motion.")
        else:
            travel_success += 1

    print(f"\n{'='*60}")
    print(f"Done.")
    print(f"  Extrusion segments completed : {extrude_success}/{extrude_segs}")
    print(f"  Travel segments completed    : {travel_success}/{travel_segs}")
    if skip_count:
        print(f"  Segments skipped            : {skip_count}")
    print(f"{'='*60}")

    klipper.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
