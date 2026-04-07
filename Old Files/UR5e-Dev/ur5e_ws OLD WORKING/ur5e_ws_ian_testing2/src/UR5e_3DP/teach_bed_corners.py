#!/usr/bin/env python3
"""
teach_bed_corners.py
====================
Interactively record the 4 bed corner positions by reading the current
robot TCP position from TF.

Usage:
  1. Start the demo:  ros2 launch ur_cell_config demo.launch.py
  2. Jog the robot TCP to each bed corner using RViz
  3. Run:  python3 teach_bed_corners.py
  4. Press ENTER at each corner to record it
  5. Copy the printed BED_CORNERS block into gcode_to_waypoints.py

Requirements: demo.launch.py must be running (TF available)
"""

import rclpy
from rclpy.node import Node
import tf2_ros


class CornerTeacher(Node):
    def __init__(self):
        super().__init__('corner_teacher')
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_tcp(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'tool_tip', rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None


def main():
    rclpy.init()
    node = CornerTeacher()

    print("\n=== Bed Corner Teaching ===")
    print("Waiting for TF to become available...")
    import time
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.2)
        time.sleep(0.1)
    print("Ready.\n")
    print("Jog the robot TCP to each bed corner in RViz, then press ENTER.\n")

    corners = []
    for i in range(1, 5):
        input(f"  Move to corner {i}, then press ENTER...")
        # Spin for 1 s to flush fresh joint states into the TF buffer
        import time
        deadline = time.time() + 1.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        pos = node.get_tcp()
        if pos is None:
            print("  ERROR: Could not read TCP position. Is the demo running?")
            rclpy.shutdown()
            return
        corners.append(pos)
        print(f"  Corner {i}: x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f}\n")

    print("\n=== Copy this into gcode_to_waypoints.py ===\n")
    print("USE_BED_LEVELING = True")
    print("BED_CORNERS = [")
    for i, (x, y, z) in enumerate(corners, 1):
        print(f"    ({x:.4f},  {y:.4f},  {z:.4f}),  # corner {i}")
    print("]")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
