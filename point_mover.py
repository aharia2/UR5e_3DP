#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, OrientationConstraint
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from shape_msgs.msg import SolidPrimitive

class PointMover(Node):
    def __init__(self):
        super().__init__('point_mover')
        
        # Cartesian path planning service
        self.compute_plan_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        # Path visualization publisher
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        
        print("Waiting for MoveIt services...")
        self.compute_plan_client.wait_for_service()
        self.execute_client.wait_for_server()
        print("Ready!")
    
    def publish_path_marker(self, points):
        """Publish a marker showing the planned path"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "point_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime.sec = 60  # Persist for 60 seconds
        
        # Line properties
        marker.scale.x = 0.0005  # 0.5mm line width
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue (different from cartesian_mover)
        
        # Add points to the line
        for pt in points:
            marker.points.append(pt)
        
        # Publish multiple times to ensure RViz receives it
        for _ in range(5):
            self.marker_pub.publish(marker)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        print(f"Published path visualization with {len(points)} points (BLUE LINE)")
    
    def point_to_pose(self, point):
        """Convert a point to a pose with downward-pointing orientation (parallel to Z-axis)"""
        pose = Pose()
        pose.position = point
        
        # Orientation: tool pointing down (parallel to -Z axis)
        # This is a nominal orientation; rotation about Z is unconstrained
        pose.orientation.x = 0.70711
        pose.orientation.y = 0.00056
        pose.orientation.z = 0.00056
        pose.orientation.w = 0.70711
        
        return pose
    
    def move_to_point_ptp(self, point):
        """Move to a point using PTP planning - tool points down, Z-rotation free"""
        print(f"\nMoving to point (PTP):")
        print(f"  Position: ({point.x:.3f}, {point.y:.3f}, {point.z:.3f})")
        
        move_client = ActionClient(self, MoveGroup, '/move_action')
        move_client.wait_for_server()
        
        goal = MoveGroup.Goal()
        goal.request.group_name = "ur_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        
        # Use simpler approach: set target pose with loose orientation constraint
        from geometry_msgs.msg import PoseStamped
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = "base_link"
        target_pose_stamped.pose = self.point_to_pose(point)
        
        # Set pose target
        goal.request.goal_constraints.append(Constraints())
        
        # Position constraint - use larger tolerance
        from moveit_msgs.msg import PositionConstraint
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool_tip"
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])  # 1cm tolerance
        )
        pc.constraint_region.primitive_poses.append(target_pose_stamped.pose)
        pc.weight = 1.0
        goal.request.goal_constraints[0].position_constraints.append(pc)
        
        # Orientation constraint with Z-axis freedom
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool_tip"
        oc.orientation = target_pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1  # Allow some tilt
        oc.absolute_y_axis_tolerance = 0.1  # Allow some tilt
        oc.absolute_z_axis_tolerance = 6.28  # Free rotation around Z
        oc.weight = 0.5  # Lower weight for more flexibility
        goal.request.goal_constraints[0].orientation_constraints.append(oc)
        
        print("Planning PTP motion...")
        future = move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            print("ERROR: PTP goal rejected")
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        if result.result.error_code.val != 1:
            print(f"ERROR: PTP failed with code {result.result.error_code.val}")
            return False
        
        print("SUCCESS: PTP motion completed!")
        return True
    
    def move_through_points_ptp(self, points):
        """Move through points using PTP planning for each segment with Z-rotation freedom"""
        print(f"\nMoving through {len(points)} points using PTP (Z-axis rotation free)...")
        
        # Visualize the path in RViz
        self.publish_path_marker(points)
        
        for i, point in enumerate(points, 1):
            print(f"\n{'='*50}")
            print(f"Moving to point {i} of {len(points)}")
            print(f"{'='*50}")
            
            success = self.move_to_point_ptp(point)
            if not success:
                print(f"Failed to reach point {i}, stopping.")
                return False
        
        print("\nSUCCESS: Completed path through all points!")
        return True


def make_point(x, y, z):
    """Helper function to create a Point from 3 values"""
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point


def main():
    rclpy.init()
    mover = PointMover()
    
    # =====================================================
    # DEFINE TARGET POINTS HERE (edit these values)
    # Format: make_point(x, y, z)
    # Tool will automatically point downward (parallel to Z-axis)
    # Tool can rotate freely about the Z-axis during motion
    # =====================================================
    import math
    
    # Square spanning quadrants - sized to fit workspace safely
    center_x = 0.25
    center_y = 0.0
    height = 0.15
    size = 0.08  # 8cm half-size = 16cm square (smaller, safer)
    
    # Square corners in all 4 quadrants
    target_points = [
        make_point(center_x + size, center_y + size, height),   # +X, +Y quadrant
        make_point(center_x - size, center_y + size, height),   # -X, +Y quadrant
        make_point(center_x - size, center_y - size, height),   # -X, -Y quadrant
        make_point(center_x + size, center_y - size, height),   # +X, -Y quadrant
        make_point(center_x + size, center_y + size, height),   # Return to start
    ]
    
    print(f"Generated square with {len(target_points)} points")
    print(f"Workspace: X({center_x-size:.2f} to {center_x+size:.2f}), Y({center_y-size:.2f} to {center_y+size:.2f}), Z={height}")
    
    # Execute motion: PTP to first point, then Cartesian for continuous path
    print(f"\n{'='*50}")
    print(f"Step 1: Moving to start position (PTP)")
    print(f"{'='*50}")
    success = mover.move_to_point_ptp(target_points[0])
    
    if not success:
        print("Failed to reach start position, stopping.")
    else:
        # Now move through remaining points using PTP (Z-rotation free)
        if len(target_points) > 1:
            print(f"\n{'='*50}")
            print(f"Step 2: Moving through remaining points (PTP)")
            print(f"{'='*50}")
            success = mover.move_through_points_ptp(target_points[1:])
            
            if success:
                # Read final pose
                from tf2_ros import Buffer, TransformListener
                from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
                import time
                
                tf_buffer = Buffer()
                tf_listener = TransformListener(tf_buffer, mover)
                time.sleep(0.5)
                
                print("\nReading final pose...")
                for j in range(5):
                    try:
                        trans = tf_buffer.lookup_transform('base_link', 'tool_tip', rclpy.time.Time())
                        p = trans.transform.translation
                        q = trans.transform.rotation
                        print(f"Final Position: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
                        print(f"Final Orientation: (x={q.x:.3f}, y={q.y:.3f}, z={q.z:.3f}, w={q.w:.3f})")
                        break
                    except (LookupException, ConnectivityException, ExtrapolationException):
                        rclpy.spin_once(mover, timeout_sec=0.1)
    
    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
