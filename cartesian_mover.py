#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class CartesianMover(Node):
    def __init__(self):
        super().__init__('cartesian_mover')
        
        # Cartesian path planning service
        self.compute_plan_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        # Path visualization publisher
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        
        print("Waiting for MoveIt services...")
        self.compute_plan_client.wait_for_service()
        self.execute_client.wait_for_server()
        print("Ready!")
    
    def publish_path_marker(self, waypoints):
        """Publish a marker showing the planned path"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cartesian_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime.sec = 60  # Persist for 60 seconds
        
        # Line properties - make it bright green with thin line
        marker.scale.x = 0.0005  # 0.5mm line width
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Bright green
        
        # Add waypoint positions to the line
        for pose in waypoints:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            marker.points.append(point)
        
        # Publish multiple times to ensure RViz receives it
        for _ in range(5):
            self.marker_pub.publish(marker)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        print(f"Published path visualization with {len(waypoints)} waypoints (GREEN LINE)")

    def move_to_pose(self, target_pose):
        """Move the robot to the specified pose using Cartesian path planning"""
        print(f"\nMoving to pose:")
        print(f"  Position: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
        print(f"  Orientation: (x={target_pose.orientation.x:.3f}, y={target_pose.orientation.y:.3f}, z={target_pose.orientation.z:.3f}, w={target_pose.orientation.w:.3f})")
        
        # Request Cartesian path
        req = GetCartesianPath.Request()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = "base_link"
        req.group_name = "ur_arm"
        req.link_name = "tool_tip"
        req.waypoints = [target_pose]
        req.max_step = 0.01  # 1cm resolution for smooth path
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        print("Planning Cartesian path...")
        future = self.compute_plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response.fraction < 0.95:
            print(f"ERROR: Only computed {response.fraction * 100:.1f}% of the Cartesian path")
            return False
        
        print(f"Cartesian path computed ({response.fraction * 100:.1f}%). Executing...")
        
        # Execute the trajectory
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        
        send_goal_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            print("ERROR: Execution rejected")
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        print("SUCCESS: Cartesian motion completed!")
        return True
    
    def move_to_pose_ptp(self, target_pose):
        """Move to pose using PTP (point-to-point) planning - uses arcs, not straight lines"""
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        
        print(f"\nMoving to pose (PTP):")
        print(f"  Position: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
        
        move_client = ActionClient(self, MoveGroup, '/move_action')
        move_client.wait_for_server()
        
        goal = MoveGroup.Goal()
        goal.request.group_name = "ur_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool_tip"
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0
        c.position_constraints.append(pc)
        
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool_tip"
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        
        goal.request.goal_constraints.append(c)
        
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


def make_pose(x, y, z, qx, qy, qz, qw):
    """Helper function to create a Pose from 7 values (position + quaternion)"""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def main():
    rclpy.init()
    mover = CartesianMover()
    
    # =====================================================
    # DEFINE TARGET POSES HERE (edit these values)
    # Format: make_pose(x, y, z, qx, qy, qz, qw)
    # =====================================================
    target_poses = [
        make_pose(0.15, 0.15, 0.01, 0.70711, 0.00056, 0.00056, 0.70711),
        make_pose(0.25, 0.15, 0.01, 0.70711, 0.00056, 0.00056, 0.70711),
        make_pose(0.25, 0.25, 0.01, 0.70711, 0.00056, 0.00056, 0.70711),
        make_pose(0.15, 0.25, 0.01, 0.70711, 0.00056, 0.00056, 0.70711),
        make_pose(0.15, 0.15, 0.01, 0.70711, 0.00056, 0.00056, 0.70711),
        
        # Add more poses here as needed:
        # make_pose(0.5, 0.0, 0.4, 0.70711, 0.00056, 0.00056, 0.70711),
    ]
    
    # Execute motion: PTP to first corner, then Cartesian for entire square
    print(f"\n{'='*50}")
    print(f"Step 1: Moving to start position (PTP)")
    print(f"{'='*50}")
    success = mover.move_to_pose_ptp(target_poses[0])
    
    if not success:
        print("Failed to reach start position, stopping.")
    else:
        # Now plan the entire square as one continuous Cartesian path
        print(f"\n{'='*50}")
        print(f"Step 2: Tracing square (Continuous Cartesian path)")
        print(f"{'='*50}")
        
        # Request Cartesian path for all remaining waypoints (the square)
        req = GetCartesianPath.Request()
        req.header.stamp = mover.get_clock().now().to_msg()
        req.header.frame_id = "base_link"
        req.group_name = "ur_arm"
        req.link_name = "tool_tip"
        req.waypoints = target_poses[1:]  # All points except the first (already there)
        req.max_step = 0.01  # 1cm resolution
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        print(f"Planning continuous path through {len(target_poses)-1} waypoints...")
        future = mover.compute_plan_client.call_async(req)
        rclpy.spin_until_future_complete(mover, future)
        response = future.result()
        
        if response.fraction < 0.95:
            print(f"ERROR: Only computed {response.fraction * 100:.1f}% of the path")
        else:
            print(f"Continuous path computed ({response.fraction * 100:.1f}%). Executing...")
            
            # Visualize the path in RViz
            mover.publish_path_marker(target_poses)
            
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = response.solution
            
            send_goal_future = mover.execute_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(mover, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                print("ERROR: Execution rejected")
            else:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(mover, result_future)
                print("\nSUCCESS: Continuous square motion completed!")
                
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
