#!/usr/bin/env python3
"""
gcode_executor.py

Moves the UR5e through waypoints, executing one straight-line Cartesian
segment at a time.  Accepts either a pre-generated CSV or a raw G-code file
(converted on-the-fly using gcode_to_waypoints settings).

Usage:
    python3 gcode_executor.py                              # default waypoints.csv
    python3 gcode_executor.py /path/to/file.csv           # custom CSV
    python3 gcode_executor.py "/path/to/print.gcode"      # G-code  ← NEW
"""

import rclpy
import csv
import os
import sys
import math
import importlib.util

# Import conversion helpers from gcode_to_waypoints (same directory)
_gtw_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gcode_to_waypoints.py')
_spec = importlib.util.spec_from_file_location('gcode_to_waypoints', _gtw_path)
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
EXTRUDE_MULTIPLIER   = 1.0   # multiplier for distance-based extrusion (mm/mm)
EXTRUDING_VELOCITY   = 0.1   # arm velocity (0.01–1.0) while EXTRUDING filament — 5% = 50 mm/s
TRAVEL_VELOCITY      = 0.1    # arm velocity (0.01–1.0) for TRAVEL (no extrusion) moves
POSITION_TOLERANCE   = 0.008  # 8 mm – acceptable error before extrusion
CARTESIAN_STEP       = 0.005  # 5 mm interpolation step along each segment
KLIPPER_FEEDRATE     = 600   # mm/min max feedrate (motor limit: 40 mm/s = 2400 mm/min)
KLIPPER_MIN_FEEDRATE = 200    # mm/min minimum Klipper feedrate clamp (prevents silent rejects)
# ───────────────────────────────────────────────────────────────────────────


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


class GCodeExecutor(Node):
    def __init__(self):
        super().__init__('gcode_executor')

        self.cart_client    = self.create_client(GetCartesianPath,
                                                  '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory,
                                           '/execute_trajectory')
        self.extrude_pub    = self.create_publisher(Float64,
                                                    '/extruder_command', 10)
        self.marker_pub     = self.create_publisher(Marker,
                                                    '/visualization_marker', 10)

        # TF2 for position verification
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        print("Waiting for MoveIt services...")
        self.cart_client.wait_for_service()
        self.execute_client.wait_for_server()

        # Let TF buffer fill before any lookups
        print("Waiting for TF data...")
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
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

    # def move_ptp(self, target_pose):
    #     """PTP approach to first waypoint."""
    #     move_client = ActionClient(self, MoveGroup, '/move_action')
    #     move_client.wait_for_server()
    #
    #     goal = MoveGroup.Goal()
    #     goal.request.group_name = "ur_arm"
    #     goal.request.num_planning_attempts = 30
    #     goal.request.allowed_planning_time  = 15.0
    #     goal.request.max_velocity_scaling_factor     = 0.3
    #     goal.request.max_acceleration_scaling_factor = 0.3
    #
    #     c  = Constraints()
    #     pc = PositionConstraint()
    #     pc.header.frame_id = "base_link"
    #     pc.link_name       = "tool_tip"
    #     sphere             = SolidPrimitive()
    #     sphere.type        = SolidPrimitive.SPHERE
    #     sphere.dimensions  = [0.01]
    #     pc.constraint_region.primitives.append(sphere)
    #     pc.constraint_region.primitive_poses.append(target_pose)
    #     pc.weight = 1.0
    #     c.position_constraints.append(pc)
    #
    #     oc = OrientationConstraint()
    #     oc.header.frame_id = "base_link"
    #     oc.link_name       = "tool_tip"
    #     oc.orientation     = target_pose.orientation
    #     oc.absolute_x_axis_tolerance = 0.5
    #     oc.absolute_y_axis_tolerance = 0.5
    #     oc.absolute_z_axis_tolerance = 6.28
    #     oc.weight = 0.5
    #     c.orientation_constraints.append(oc)
    #
    #     goal.request.goal_constraints.append(c)
    #
    #     future = move_client.send_goal_async(goal)
    #     rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
    #     if not future.done():
    #         print("ERROR: PTP goal send timed out")
    #         return False
    #     gh = future.result()
    #     if not gh.accepted:
    #         print("ERROR: PTP goal rejected")
    #         return False
    #
    #     rf = gh.get_result_async()
    #     rclpy.spin_until_future_complete(self, rf, timeout_sec=30.0)
    #     if not rf.done():
    #         print("ERROR: PTP result timed out")
    #         return False
    #     result = rf.result()
    #     if result.result.error_code.val != 1:
    #         print(f"ERROR: PTP failed (code {result.result.error_code.val})")
    #         return False
    #
    #     print("SUCCESS: PTP approach complete")
    #     return True

    def get_final_robot_state(self, trajectory):
        """Extract the final RobotState from a planned trajectory for use as a start state."""
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        last_pt = trajectory.joint_trajectory.points[-1]
        rs = RobotState()
        rs.joint_state.name     = list(trajectory.joint_trajectory.joint_names)
        rs.joint_state.position = list(last_pt.positions)
        rs.joint_state.velocity = list(last_pt.velocities) if last_pt.velocities else []
        return rs

    def plan_cartesian_segment(self, end_pose, velocity_scale=TRAVEL_VELOCITY,
                               start_state=None):
        """
        Plan a single straight-line Cartesian segment.
        start_state: RobotState to plan from (None = current robot state).
        Returns the RobotTrajectory, or None on failure.
        """
        req = GetCartesianPath.Request()
        req.header.stamp    = self.get_clock().now().to_msg()
        req.header.frame_id = "base_link"
        req.group_name      = "ur_arm"
        req.link_name       = "tool_tip"
        req.waypoints       = [end_pose]
        req.max_step        = CARTESIAN_STEP
        req.jump_threshold  = 4.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor     = float(velocity_scale)
        req.max_acceleration_scaling_factor = float(velocity_scale)
        if start_state is not None:
            req.start_state = start_state

        future = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done():
            print("  WARNING: CartesianPath planning timed out (15s)")
            return None
        response = future.result()

        if response.fraction < 0.95:
            print(f"  WARNING: Segment only {response.fraction*100:.1f}% computed")
            return None
        return response.solution


    def execute_trajectory(self, trajectory, timeout_sec=30.0,
                           klipper=None, extrude_mm=0.0, velocity_scale=1.0, extrude_pub=None):
        """Execute a pre-planned RobotTrajectory. Fires Klipper extrusion right before motion starts."""
        # Estimate a reasonable timeout from trajectory duration + buffer
        pts = trajectory.joint_trajectory.points
        if pts:
            last_t = pts[-1].time_from_start
            # MoveIt already scales timestamps to match velocity_scale — use directly
            traj_secs = last_t.sec + last_t.nanosec * 1e-9
            timeout_sec = max(timeout_sec, traj_secs * 3.0 + 5.0)
        else:
            traj_secs = 0.0

        # Calculate feedrate so Klipper finishes extrusion when arm arrives,
        # but clamp to [KLIPPER_MIN_FEEDRATE, KLIPPER_FEEDRATE] to protect the extruder motor
        if extrude_mm > 0 and traj_secs > 0:
            feedrate = (extrude_mm / traj_secs) * 60.0
            feedrate = max(KLIPPER_MIN_FEEDRATE, min(KLIPPER_FEEDRATE, feedrate))
        else:
            feedrate = KLIPPER_FEEDRATE

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        # Fire extrusion in a BACKGROUND THREAD — Moonraker's /printer/gcode/script
        # is synchronous (blocks until Klipper finishes). Running it in a thread
        # lets the arm start moving immediately at the same time.
        if extrude_mm > 0:
            if extrude_pub is not None:
                from std_msgs.msg import Float64
                msg = Float64()
                msg.data = extrude_mm
                extrude_pub.publish(msg)
            if klipper is not None:
                import threading
                MAX_CHUNK_MM = 50.0

                # Split into ≤50mm chunks.  Each send_extrusion() call blocks until
                # Klipper finishes that chunk, so sequencing them naturally gives
                # gapless, continuously-timed extrusion.
                chunks = []
                remaining = extrude_mm
                while remaining > 1e-6:
                    chunk = min(MAX_CHUNK_MM, remaining)
                    chunks.append(chunk)
                    remaining -= chunk

                # feedrate is per-chunk (mm/min) — each chunk runs at the same rate
                # so total duration = sum(chunk/feedrate*60) = extrude_mm/feedrate*60 = traj_secs
                print(f"  Extruding {extrude_mm:.2f} mm in {len(chunks)} chunk(s) "
                      f"at {feedrate:.0f} mm/min over {traj_secs:.1f}s")

                def _send_chunks(klipper, chunks, feedrate):
                    for chunk in chunks:
                        klipper.send_extrusion(chunk, feedrate=feedrate)

                t = threading.Thread(target=_send_chunks,
                                     args=(klipper, chunks, feedrate),
                                     daemon=True)
                t.start()

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
            print(f"  ERROR: Execution timed out after {timeout_sec:.0f}s — skipping segment")
            return False
        return True


    # ── Extrusion ─────────────────────────────────────────────────────────

    def trigger_extrusion(self, amount_mm, waypoint_index, target_pose,
                          klipper: KlipperComm = None):
        """
        Verify position, publish ROS /extruder_command, and send Klipper command.
        Returns True if within tolerance and commands were sent.
        """
        # Let TF settle for a moment
        rclpy.spin_once(self, timeout_sec=0.1)

        err = self.distance_to_target(target_pose)
        pos = self.get_tcp_position()

        if pos:
            print(f"  TCP: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})  "
                  f"error: {err*1000:.1f} mm")

        if err <= POSITION_TOLERANCE:
            # Publish ROS topic
            msg = Float64()
            msg.data = amount_mm
            self.extrude_pub.publish(msg)

            # Send to Klipper
            if klipper is not None:
                klipper.send_extrusion(amount_mm, feedrate=KLIPPER_FEEDRATE)

            print(f"  ✓ EXTRUDE {amount_mm:.2f} mm  (waypoint {waypoint_index})")
            return True
        else:
            print(f"  ✗ Skipped extrusion — position error {err*1000:.1f} mm "
                  f"> tolerance {POSITION_TOLERANCE*1000:.1f} mm")
            return False

    # ── Visualization ─────────────────────────────────────────────────────

    def publish_remaining_path(self, poses, current_idx):
        """Show remaining path as a blue line in RViz."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.ns   = "gcode_path"
        marker.id   = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.002
        marker.color = ColorRGBA(r=0.0, g=0.4, b=1.0, a=0.8)
        from geometry_msgs.msg import Point
        for p in poses[current_idx:]:
            pt = Point()
            pt.x = p.position.x
            pt.y = p.position.y
            pt.z = p.position.z
            marker.points.append(pt)
        self.marker_pub.publish(marker)


# ── Main ──────────────────────────────────────────────────────────────────

def load_waypoints(csv_file):
    """Load poses, optional extrusion amounts, and optional velocity scales from CSV."""
    poses    = []
    extrudes = []
    velocities = []
    with open(csv_file, 'r') as f:
        for row in csv.reader(f):
            row = [c.strip() for c in row if c.strip()]
            if len(row) < 7:
                continue
            try:
                x, y, z, qx, qy, qz, qw = [float(v) for v in row[:7]]
                poses.append(make_pose(x, y, z, qx, qy, qz, qw))
                
                if len(row) >= 8:
                    e = float(row[7])
                else:
                    if len(poses) > 1:
                        prev = poses[-2]
                        dx = x - prev.position.x
                        dy = y - prev.position.y
                        dz = z - prev.position.z
                        dist_mm = math.sqrt(dx*dx + dy*dy + dz*dz) * 1000.0
                        e = dist_mm * EXTRUDE_MULTIPLIER
                    else:
                        e = 0.0

                v = float(row[8]) if len(row) >= 9 else (EXTRUDING_VELOCITY if e > 0 else TRAVEL_VELOCITY)
                extrudes.append(e)
                velocities.append(max(0.01, min(1.0, v)))  # clamp 0.01–1.0
            except ValueError:
                pass  # skip header / comment lines
    return poses, extrudes, velocities


def main():
    rclpy.init()
    node = GCodeExecutor()

    # ── Connect to Klipper ───────────────────────────────────────────────
    klipper = KlipperComm()

    # ── Load waypoints ──────────────────────────────────────────────────
    default_csv = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'waypoints.csv')
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
        print(f"  {len(raw)} raw moves → {len(filtered)} waypoints after downsampling")

        # Build poses / extrudes / velocities directly from the module
        poses     = []
        extrudes  = []
        velocities = []
        for p in filtered:
            rx, ry, rz = _gcode_mod.gcode_to_robot(p['x'], p['y'], p['z'])
            poses.append(make_pose(rx, ry, rz,
                                   _gcode_mod.QX, _gcode_mod.QY,
                                   _gcode_mod.QZ, _gcode_mod.QW))
            e = p['e_delta'] if _gcode_mod.EXTRUDE_MM == 'auto' else float(_gcode_mod.EXTRUDE_MM)
            extrudes.append(e)
            velocities.append(EXTRUDING_VELOCITY if e > 0 else TRAVEL_VELOCITY)
        csv_file = input_file   # just for display
    else:
        poses, extrudes, velocities = load_waypoints(input_file)
        csv_file = input_file

    if len(poses) < 2:
        print("ERROR: Need at least 2 waypoints")
        rclpy.shutdown()
        return

    print(f"Loaded {len(poses)} waypoints from: {csv_file}")
    print(f"Position tolerance: {POSITION_TOLERANCE*1000:.1f} mm")
    print(f"Extrude multiplier: {EXTRUDE_MULTIPLIER} (distance × multiplier)")
    print(f"Extruding velocity: {EXTRUDING_VELOCITY*100:.0f}% of max  |  Travel velocity: {TRAVEL_VELOCITY*100:.0f}% of max")

    # ── Step 1: Cartesian approach to first waypoint ─────────────────────
    print(f"\n{'='*55}")
    print(f"Step 1: Cartesian approach to waypoint 0")
    print(f"{'='*55}")
    cur = node.get_tcp_position()
    if cur:
        print(f"  Current: ({cur[0]*1000:.1f}, {cur[1]*1000:.1f}, {cur[2]*1000:.1f}) mm")
    else:
        print(f"  Current: (unknown — TF not available)")
    print(f"  Target:  ({poses[0].position.x*1000:.1f}, "
                       f"{poses[0].position.y*1000:.1f}, "
                       f"{poses[0].position.z*1000:.1f}) mm")

    try:
        ans = input("\nPress ENTER to move robot to start position, or 'q' + ENTER to quit: ")
    except EOFError:
        # stdin is not interactive (e.g. launched via ros2 run / launch file)
        # — abort instead of silently proceeding.
        print("ERROR: No interactive stdin available. Aborting for safety.")
        klipper.close()
        rclpy.shutdown()
        return
    if ans.strip().lower() == 'q':
        print("Aborted by user.")
        klipper.close()
        rclpy.shutdown()
        return

    traj0 = node.plan_cartesian_segment(poses[0], TRAVEL_VELOCITY, start_state=None)
    if traj0 is not None:
        ok0 = node.execute_trajectory(traj0)
    else:
        ok0 = False

    if not ok0:
        print("ERROR: Cartesian approach to start position failed.")
        klipper.close()
        rclpy.shutdown()
        return

    node.trigger_extrusion(extrudes[0], 0, poses[0], klipper)

    # ── Step 2: Pre-plan ALL segments ────────────────────────────────────
    # Plan every segment upfront (chained via trajectory end states) so
    # there is zero planning lag during execution.
    print(f"\n{'='*55}")
    print(f"Step 2: Pre-planning {len(poses)-1} Cartesian segments...")
    print(f"{'='*55}")

    trajectories = []
    start_state  = None   # None = use current real robot state for first seg

    for i in range(1, len(poses)):
        target = poses[i]
        print(f"  Planning segment {i}/{len(poses)-1}  "
              f"vel={velocities[i]*100:.0f}%  ...", end="", flush=True)

        traj = node.plan_cartesian_segment(target, velocities[i], start_state)
        if traj is None:
            print(f"\n  ERROR: Could not plan segment {i}, aborting.")
            klipper.close()
            rclpy.shutdown()
            return

        print(" OK")
        trajectories.append(traj)
        # Chain: next segment starts where this one ends
        start_state = node.get_final_robot_state(traj)

    # ── Estimate total execution time ────────────────────────────────────
    total_traj_secs = 0.0
    for traj in trajectories:
        pts = traj.joint_trajectory.points
        if pts:
            last_t = pts[-1].time_from_start
            total_traj_secs += last_t.sec + last_t.nanosec * 1e-9

    # Add ~100 ms position-verification pause per waypoint
    verification_secs = len(trajectories) * 0.1
    total_secs = total_traj_secs + verification_secs

    hours   = int(total_secs // 3600)
    minutes = int((total_secs % 3600) // 60)
    seconds = int(total_secs % 60)

    print(f"\n{'='*55}")
    print(f"Planning complete — {len(trajectories)} segments ready")
    print(f"  Trajectory time : {total_traj_secs/60:.1f} min")
    print(f"  Verification    : {verification_secs/60:.1f} min")
    print(f"  Estimated total : {hours:02d}h {minutes:02d}m {seconds:02d}s")
    print(f"{'='*55}")

    node.publish_remaining_path(poses, 0)

    # ── Wait for user confirmation before executing ───────────────────────
    try:
        ans = input("\nPress ENTER to start execution, or type 'q' + ENTER to quit: ")
    except EOFError:
        print("ERROR: No interactive stdin available. Aborting for safety.")
        klipper.close()
        rclpy.shutdown()
        return
    if ans.strip().lower() == 'q':
        print("Aborted by user.")
        klipper.close()
        rclpy.shutdown()
        return

    # ── Step 3: Execute all segments, trigger extrusion between each ──────
    print(f"\n{'='*55}")
    print(f"Step 3: Executing {len(trajectories)} segments")
    print(f"{'='*55}")

    success_count = 0
    skip_count    = 0
    for i, traj in enumerate(trajectories):
        waypoint_idx = i + 1
        target = poses[waypoint_idx]
        
        print(f"\nExecuting segment {waypoint_idx}/{len(poses)-1}: "
              f"({target.position.x:.4f}, "
               f"{target.position.y:.4f}, "
               f"{target.position.z:.4f})")

        ok = node.execute_trajectory(
            traj,
            klipper=klipper,
            extrude_mm=extrudes[waypoint_idx],
            velocity_scale=velocities[waypoint_idx],
            extrude_pub=node.extrude_pub
        )

        if not ok:
            print(f"  WARNING: Execution failed for segment {waypoint_idx} — skipping.")
            skip_count += 1
            continue

        if extrudes[waypoint_idx] > 0:
            success_count += 1


    print(f"\n{'='*55}")
    print(f"Done. {success_count}/{len(poses)-1} extrusion commands sent.")
    if skip_count:
        print(f"  ({skip_count} segments skipped due to execution timeout)")
    print(f"{'='*55}")

    klipper.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
