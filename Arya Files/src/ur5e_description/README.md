# UR5e Description

This package contains the URDF description and launch files for the UR5e robot.

## Visualizing the Robot

To visualize the UR5e robot in RViz with the Joint State Publisher GUI, follow these steps:

1.  **Source ROS 2 Environment** (if not already in your `.bashrc`):
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```

2.  **Source the Workspace**:
    Navigate to your workspace root (e.g., `~/ROS/UR5e`) and run:
    ```bash
    source install/setup.bash
    ```

3.  **Launch the Simulation**:
    Run the following launch file to start `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`:
    ```bash
    ros2 launch ur5e_description simulate.launch.py
    ```

You should see the RViz window open with the robot model, and a separate GUI window to control the joint angles.
