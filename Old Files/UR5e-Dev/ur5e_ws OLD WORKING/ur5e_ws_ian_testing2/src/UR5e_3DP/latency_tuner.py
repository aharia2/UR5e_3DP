#!/usr/bin/env python3
"""
latency_tuner.py
================
Moves the robot a short straight line while extruding, so you can
empirically tune KLIPPER_START_LATENCY in gcode_executor_continuous_NEW.py.

What it does
------------
1. Reads the current TCP position from TF.
2. Plans a short Cartesian move (MOVE_DISTANCE_M in the +X direction by default).
3. Builds a per-interval extrusion schedule (same logic as the NEW executor).
4. Fires extrusion commands with TEST_LATENCY applied to the first command.
5. Prints wall-clock timestamps for every event so you can compare what you
   see physically with the log.

How to tune
-----------
- Run the script and watch the extruder gear and the robot arm.
- If the extruder starts moving BEFORE the robot → increase TEST_LATENCY.
- If the robot starts moving BEFORE the extruder → decrease TEST_LATENCY (or go negative).
- Once they start together, copy the value into KLIPPER_START_LATENCY in
  gcode_executor_continuous_NEW.py.

Usage
-----
    python3 latency_tuner.py              # uses defaults below
    python3 latency_tuner.py 0.03         # override TEST_LATENCY from CLI
"""

import sys
import math
import time as _time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64
import tf2_ros

from Klipper_communcation import KlipperComm

# ─── Tuning target ────────────────────────────────────────────────────────────
# Edit this value, re-run, watch the hardware, repeat.
# Copy the final value into KLIPPER_START_LATENCY in gcode_executor_continuous_NEW.py
TEST_LATENCY = 0.05          # seconds — positive = delay Klipper, negative = pre-send

# ─── Test move parameters ─────────────────────────────────────────────────────
MOVE_DISTANCE_M  = 0.030     # metres — length of the test move (30 mm default)
MOVE_DIRECTION   = (1, -1, 0) # (dx, dy, dz) unit vector — +X by default
VELOCITY_SCALE   = 0.04      # match your print velocity scale
EXTRUDE_MM       = 10        # total filament to extrude over the move (mm)

# ─── Burst mode ────────────────────────────────────────────────────────────────
# When True: ignores the per-interval schedule and fires ONE large extrusion
# command at t=TEST_LATENCY.  The gear will spin for BURST_EXTRUDE_MM at
# BURST_FEEDRATE mm/min — very easy to see start relative to robot motion.
# Use this to tune TEST_LATENCY, then switch back to False for the real print.
BURST_MODE         = True
BURST_EXTRUDE_MM   = 15.0    # mm — enough for several full gear rotations
BURST_FEEDRATE     = 300.0   # mm/min — fast enough to see clearly

# ─── TF frame names ───────────────────────────────────────────────────────────
# If TF lookup fails, check available frames with:  ros2 run tf2_tools view_frames
BASE_FRAME = 'base_link'     # robot base frame
TCP_FRAME  = 'tool_tip'      # tool tip frame

# ─── Manual start position fallback ───────────────────────────────────────────
# If TF is unavailable, set USE_MANUAL_POSE = True and fill in the current
# TCP position (metres) and orientation (quaternion xyzw) here.
USE_MANUAL_POSE   = False
MANUAL_POSE_XYZ   = (0.5, 0.0, 0.08)       # metres in base_link
MANUAL_POSE_QXYZW = (0.5, 0.5, 0.5, 0.5)   # nozzle-down orientation
# ─────────────────────────────────────────────────────────────────────────────


def _ts():
    """Wall-clock timestamp string for logging."""
    return f"{_time.monotonic():.4f}s"


def make_pose(x, y, z, qx, qy, qz, qw):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


def _build_schedule(traj, total_extrude):
    """
    Build a time-indexed extrusion schedule from a planned joint trajectory.
    Identical to _build_extrusion_schedule() in gcode_executor_continuous_NEW.py.
    """
    pts = traj.joint_trajectory.points
    if len(pts) < 2:
        return []

    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
             for p in pts]

    speeds = []
    for p in pts:
        if p.velocities:
            speeds.append(math.sqrt(sum(v * v for v in p.velocities)))
        else:
            speeds.append(0.0)

    intervals = []
    total_effort = 0.0
    for i in range(1, len(pts)):
        dt = times[i] - times[i - 1]
        if dt < 1e-6:
            continue
        avg_speed = (speeds[i] + speeds[i - 1]) * 0.5
        effort = avg_speed * dt
        total_effort += effort
        intervals.append((times[i - 1], dt, effort))

    if total_effort < 1e-9 or not intervals:
        return []

    e_per_effort = total_extrude / total_effort
    schedule = []
    for (t_start, dt, effort) in intervals:
        e_i = e_per_effort * effort
        if e_i < 1e-6:
            continue
        feedrate_i = (e_i / dt) * 60.0
        schedule.append((t_start, e_i, feedrate_i))

    return schedule


class LatencyTuner(Node):
    def __init__(self):
        super().__init__('latency_tuner')
        self.cart_client    = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.extrude_pub    = self.create_publisher(Float64, '/extruder_command', 10)
        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer, self)

        print("Waiting for MoveIt services...")
        self.cart_client.wait_for_service()
        self.execute_client.wait_for_server()
        print("Ready.")

    def get_tcp_pose(self):
        """
        Return current TCP pose in BASE_FRAME.
        Falls back to MANUAL_POSE if USE_MANUAL_POSE=True or TF is unavailable.
        """
        if USE_MANUAL_POSE:
            x, y, z = MANUAL_POSE_XYZ
            qx, qy, qz, qw = MANUAL_POSE_QXYZW
            print(f"  [TF] Using manual pose: ({x}, {y}, {z})")
            return make_pose(x, y, z, qx, qy, qz, qw)

        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, TCP_FRAME, rclpy.time.Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            return make_pose(t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({BASE_FRAME} → {TCP_FRAME}): {e}")
            print("\n  TF unavailable. Options:")
            print("  1) Check available frames:  ros2 run tf2_tools view_frames")
            print(f"  2) Set USE_MANUAL_POSE = True and fill MANUAL_POSE_XYZ in latency_tuner.py")
            print(f"  3) Correct BASE_FRAME / TCP_FRAME at the top of the file")
            return None

    def plan_cartesian(self, poses, vel):
        req = GetCartesianPath.Request()
        req.header.stamp     = self.get_clock().now().to_msg()
        req.header.frame_id  = BASE_FRAME
        req.group_name       = 'ur_arm'
        req.link_name        = TCP_FRAME
        req.waypoints        = poses
        req.max_step         = 0.005
        req.jump_threshold   = 4.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor     = float(vel)
        req.max_acceleration_scaling_factor = float(vel)

        future = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            print("  ERROR: Cartesian planning timed out.")
            return None
        resp = future.result()
        if resp.fraction < 0.95:
            print(f"  ERROR: Only {resp.fraction*100:.1f}% of path computed.")
            return None
        return resp.solution

    def execute(self, traj, timeout_sec=30.0):
        pts = traj.joint_trajectory.points
        if pts:
            last_t = pts[-1].time_from_start
            traj_secs = last_t.sec + last_t.nanosec * 1e-9
            timeout_sec = max(timeout_sec, traj_secs * 3.0 + 5.0)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj

        sf = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sf, timeout_sec=10.0)
        if not sf.done() or not sf.result().accepted:
            print("  ERROR: Trajectory rejected or timed out.")
            return False

        rf = sf.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout_sec)
        return rf.done()

    def run_test(self, latency, klipper):
        print(f"\n{'='*60}")
        print(f"  TEST_LATENCY = {latency:+.3f} s")
        print(f"  Move: {MOVE_DISTANCE_M*1000:.0f} mm  |  "
              f"Extrude: {EXTRUDE_MM:.3f} mm  |  "
              f"Vel scale: {VELOCITY_SCALE}")
        print(f"{'='*60}")

        # ── Get current TCP pose ──────────────────────────────────────────
        # Spin for up to 3 s to let the TF buffer fill with live transforms
        print("  Waiting for TF...", end="", flush=True)
        deadline = _time.monotonic() + 3.0
        start_pose = None
        while _time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            start_pose = self.get_tcp_pose()
            if start_pose is not None:
                break
        if start_pose is None:
            print(" FAILED.")
        else:
            print(" OK")
        if start_pose is None:
            print("ERROR: Cannot read TCP position.")
            return

        dx, dy, dz = [v * MOVE_DISTANCE_M for v in MOVE_DIRECTION]
        end_pose = make_pose(
            start_pose.position.x + dx,
            start_pose.position.y + dy,
            start_pose.position.z + dz,
            start_pose.orientation.x,
            start_pose.orientation.y,
            start_pose.orientation.z,
            start_pose.orientation.w,
        )

        print(f"  Start : ({start_pose.position.x:.4f}, "
              f"{start_pose.position.y:.4f}, {start_pose.position.z:.4f})")
        print(f"  End   : ({end_pose.position.x:.4f}, "
              f"{end_pose.position.y:.4f}, {end_pose.position.z:.4f})")

        # ── Plan ──────────────────────────────────────────────────────────
        print("  Planning...", end="", flush=True)
        traj = self.plan_cartesian([end_pose], VELOCITY_SCALE)
        if traj is None:
            print(" FAILED.")
            return
        pts = traj.joint_trajectory.points
        traj_secs = (pts[-1].time_from_start.sec +
                     pts[-1].time_from_start.nanosec * 1e-9) if pts else 0.0
        print(f" OK  ({len(pts)} points, {traj_secs:.2f}s planned)")

        # ── Build extrusion schedule ──────────────────────────────────────
        if BURST_MODE:
            print(f"  BURST MODE: single {BURST_EXTRUDE_MM:.1f} mm command "
                  f"@ {BURST_FEEDRATE:.0f} mm/min fired at t=+{latency:.3f}s")
            print(f"  Watch: extruder gear should start spinning when robot starts moving.")
        else:
            schedule = _build_schedule(traj, EXTRUDE_MM)
            if not schedule:
                print("  WARNING: Schedule empty — joint velocities not populated.")
                print("           Falling back to single command.")
                schedule = [(0.0, EXTRUDE_MM, (EXTRUDE_MM / traj_secs) * 60.0 if traj_secs > 0 else 600.0)]
            print(f"  Schedule: {len(schedule)} commands spanning {schedule[-1][0]:.3f}s")

        # ── Confirm ───────────────────────────────────────────────────────
        try:
            ans = input("\n  Press ENTER to execute, or 'q' to quit: ")
        except EOFError:
            ans = ""
        if ans.strip().lower() == 'q':
            print("  Skipped.")
            return

        # ── Fire extrusion commands with latency compensation ─────────────
        timers = []
        t_origin = _time.monotonic()  # reference for all timer prints

        def _send(e_mm, fr, label):
            now = _time.monotonic() - t_origin
            msg = Float64()
            msg.data = e_mm
            self.extrude_pub.publish(msg)
            if klipper is not None:
                klipper.send_extrusion(e_mm, feedrate=fr)
            print(f"  [{_ts()}] EXTRUDE fired  label={label}  "
                  f"e={e_mm:.5f}mm  fr={fr:.1f}mm/min  "
                  f"(t+{now:.4f}s from origin)")

        if BURST_MODE:
            # Single burst: one large command fired at t=latency
            if latency <= 0.0:
                # Negative latency: send burst first, then wait before starting robot
                pre_move_wait = abs(latency)
                _send(BURST_EXTRUDE_MM, BURST_FEEDRATE, "BURST pre-move")
                if pre_move_wait > 0.0:
                    print(f"  Waiting {pre_move_wait:.3f}s for Klipper to spin up before robot moves...")
                    _time.sleep(pre_move_wait)
            else:
                t0 = threading.Timer(latency, _send,
                                     args=(BURST_EXTRUDE_MM, BURST_FEEDRATE,
                                           f"BURST t+{latency:.3f}s"))
                t0.start()
                timers.append(t0)
        else:
            first_t, first_e, first_fr = schedule[0]
            first_delay = first_t + latency

            if first_delay <= 0.0:
                _send(first_e, first_fr, "t=0 immediate")
            else:
                t0 = threading.Timer(first_delay, _send,
                                     args=(first_e, first_fr, f"delayed+{first_delay:.3f}s"))
                t0.start()
                timers.append(t0)

            # Record wall time just before execute so remaining timers are
            # relative to this moment (same as in gcode_executor_continuous_NEW.py)
            t_wall = _time.monotonic()
            print(f"  [{_ts()}] execute_trajectory() called  "
                  f"(t+{t_wall - t_origin:.4f}s from origin)")

            for (t_offset, e_i, fr_i) in schedule[1:]:
                elapsed = _time.monotonic() - t_wall
                delay = max(0.0, t_offset - elapsed)
                timer = threading.Timer(delay, _send,
                                        args=(e_i, fr_i, f"t={t_offset:.3f}s"))
                timer.start()
                timers.append(timer)

        t_wall = _time.monotonic()
        print(f"  [{_ts()}] execute_trajectory() called  "
              f"(t+{t_wall - t_origin:.4f}s from origin)")
        ok = self.execute(traj)

        t_end = _time.monotonic()
        print(f"  [{_ts()}] execute_trajectory() returned  ok={ok}  "
              f"(t+{t_end - t_origin:.4f}s from origin)")

        if not ok:
            cancelled = sum(1 for t in timers if t.cancel())
            print(f"  Cancelled {cancelled} pending timers.")
        else:
            # Wait for any trailing timers to finish
            _time.sleep(0.2)

        print(f"\n  Done. If extrusion led the robot: INCREASE TEST_LATENCY (currently {latency:+.3f}s)")
        print(f"         If robot led the extrusion : DECREASE TEST_LATENCY")


def main():
    rclpy.init()
    node = LatencyTuner()
    klipper = KlipperComm()

    # Allow CLI override: python3 latency_tuner.py 0.03
    latency = float(sys.argv[1]) if len(sys.argv) > 1 else TEST_LATENCY

    node.run_test(latency, klipper)

    klipper.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
