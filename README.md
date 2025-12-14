# ee_control_py

A ROS2 Python package for controlling the end-effector of a robot using MoveIt2.

## Description

This package provides a simple node that allows users to input target positions (x, y, z) for the robot's end-effector via the command line. It uses MoveIt2's motion planning to move the end-effector to the specified position with a fixed orientation.

## Requirements

- ROS2 (Humble or later)
- MoveIt2
- Python 3
- tf-transformations

## Installation

1. Clone this repository into your ROS2 workspace's `src` directory:
   ```
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/ee_control_py.git
   ```

2. Build the package:
   ```
   cd ~/ros2_ws
   colcon build --packages-select ee_control_py
   ```

3. Source the setup file:
   ```
   source install/setup.bash
   ```

## Usage

1. Launch your robot's MoveIt2 setup (e.g., for Doosan robot):
   ```
   ros2 launch dsr_controller2 dsr_moveit2.launch.py
   ```

2. Run the node:
   ```
   ros2 run ee_control_py move_ee
   ```

3. Enter the target position in meters (x y z), e.g.:
   ```
   target(x y z)> 0.30 0.00 0.40
   ```

   Type `q` or `quit` to exit.

## Configuration

- **Planning Group**: `manipulator`
- **Base Frame**: `base_link`
- **End-Effector Link**: `link_6`
- **Fixed Orientation**: Roll=180°, Pitch=0°, Yaw=0°
- **Position Tolerance**: 0.02 m
- **Orientation Tolerance**: 0.2 rad

Modify the code in `ee_control_py/move_ee.py` to adjust these parameters.

## License

MIT License