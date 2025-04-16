# UR3 Robot Pick and Place Simulation

## Project Overview

This project implements a smooth and realistic simulation of a Universal Robots UR3 robotic arm performing a pick and place operation. The simulation uses ROS2 (Robot Operating System 2) and visualizes the robot's movements in RViz with proper 3D mesh models.

## Technical Details

### Robot Model

The simulation uses the Universal Robots UR3 collaborative robot, which is a 6-DOF (Degrees of Freedom) robotic arm with the following specifications:

- **Payload**: 3 kg
- **Reach**: 500 mm
- **Joint Ranges**:
  - Base: ±360°
  - Shoulder: ±360°
  - Elbow: ±360°
  - Wrist 1: ±360°
  - Wrist 2: ±360°
  - Wrist 3: ±360°
- **Weight**: 11 kg
- **Footprint**: Ø128 mm

The UR3 is particularly suitable for light assembly tasks, pick and place operations, and applications requiring precise movements in confined spaces.

### URDF Model

The robot is defined using URDF (Unified Robot Description Format) with Xacro macros. The URDF file includes:

1. **Link Definitions**: Each segment of the robot (base, shoulder, upper arm, forearm, wrist 1, wrist 2, wrist 3, and tool0)
2. **Joint Definitions**: The connections between links, including joint types, limits, and dynamics
3. **Visual Meshes**: 3D models (.dae files) for visualization
4. **Collision Meshes**: Simplified 3D models (.stl files) for collision detection
5. **Material Properties**: Colors and textures for visual appearance

The URDF file uses absolute file paths to ensure the mesh files are correctly loaded in RViz:

```xml
<mesh filename="file:///home/lachu/ros2_workspaces/ros2_ws/src/ur3_pick_place/meshes/visual/base.dae"/>
```

### Motion Planning and Trajectory Generation

The robot's motion is controlled using a custom Python node that implements advanced trajectory generation techniques:

1. **Minimum Jerk Trajectory**: The motion follows a 5th-order polynomial trajectory that minimizes jerk (rate of change of acceleration), resulting in extremely smooth motion with zero velocity and acceleration at endpoints.

2. **Key Pose Interpolation**: The robot moves through a sequence of predefined key poses that define the pick and place operation. The motion between poses is interpolated using the minimum jerk trajectory.

3. **Dwell Time**: The robot pauses at each key pose for a configurable amount of time, which gives it time to settle and eliminates potential glitches during rapid direction changes.

4. **Angle Wrapping Handling**: The trajectory generator properly handles angle wrapping for revolute joints to ensure the robot always takes the shortest path between poses.

The mathematical formula for the minimum jerk trajectory is:

```
s(t) = 10(t/T)³ - 15(t/T)⁴ + 6(t/T)⁵
```

Where:
- `s(t)` is the normalized position at time `t`
- `T` is the total duration of the motion

This trajectory has the following properties:
- `s(0) = 0`, `s(T) = 1` (starts at 0, ends at 1)
- `s'(0) = 0`, `s'(T) = 0` (zero velocity at endpoints)
- `s''(0) = 0`, `s''(T) = 0` (zero acceleration at endpoints)

### ROS2 Architecture

The simulation uses the following ROS2 components:

1. **Nodes**:
   - `robot_state_publisher`: Publishes the robot's state (joint positions) to TF2
   - `smooth_robot_mover`: Custom node that generates and publishes smooth joint trajectories

2. **Topics**:
   - `/joint_states`: Joint positions published by the robot mover
   - `/robot_description`: URDF description of the robot
   - `/tf` and `/tf_static`: Transform frames published by the robot state publisher

3. **Parameters**:
   - `robot_description`: URDF string parameter used by the robot state publisher

### Visualization

The robot is visualized in RViz, which provides a 3D view of the robot and its environment. The visualization includes:

1. **Robot Model**: The 3D mesh models of the robot's links
2. **TF Frames**: The coordinate frames of each link
3. **Joint State Information**: The current position of each joint

### Pick and Place Sequence

The simulated pick and place operation consists of the following steps:

1. **Home Position**: The robot starts in its home position
2. **Pre-Grasp Approach**: The robot moves to a position above the object
3. **Grasp Position**: The robot lowers to the object's position
4. **Grasp**: The robot closes its gripper (simulated by wrist rotation)
5. **Lift**: The robot lifts the object
6. **Transport**: The robot moves the object to the target location
7. **Place Position**: The robot lowers the object to the target surface
8. **Release**: The robot opens its gripper (simulated by wrist rotation)
9. **Post-Place Retreat**: The robot lifts away from the placed object
10. **Return to Home**: The robot returns to its home position

## Implementation Details

### Thread Safety

The implementation uses thread locks to ensure thread safety when accessing shared data between the control loop and the ROS2 callback functions.

### Error Handling

The code includes error handling for:
- Joint limit violations
- Angle wrapping issues
- Timing inconsistencies

### Performance Optimization

Several optimizations are implemented to ensure smooth motion:

1. **High Control Rate**: The control loop runs at 100 Hz for ultra-smooth motion
2. **Precomputed Trajectories**: Trajectory parameters are precomputed to minimize computational load during execution
3. **Efficient Angle Calculations**: Optimized algorithms for calculating the shortest path between joint angles

## Future Enhancements

Potential enhancements to the simulation include:

1. **Collision Detection**: Adding collision detection to avoid obstacles
2. **Dynamic Object Interaction**: Simulating the physics of object interaction
3. **Path Planning**: Implementing advanced path planning algorithms like RRT or PRM
4. **Inverse Kinematics**: Adding inverse kinematics for end-effector positioning
5. **Gazebo Integration**: Extending the simulation to Gazebo for physics-based simulation
6. **MoveIt Integration**: Using MoveIt for advanced motion planning

## Conclusion

This project demonstrates a smooth and realistic simulation of a UR3 robot performing a pick and place operation. The implementation focuses on generating ultra-smooth motion using advanced trajectory generation techniques, while visualizing the robot with accurate 3D mesh models in RViz.

# UR3 Robot Simulation: Setup and Usage Guide

## Requirements

### Hardware Requirements
- Computer with Ubuntu 22.04 or later
- Minimum 4GB RAM (8GB recommended)
- Graphics card with OpenGL support

### Software Requirements
- ROS2 Jazzy Jalisco (or compatible version)
- Python 3.12
- Xacro
- RViz2

## Package Dependencies

The following ROS2 packages are required:
- `robot_state_publisher`
- `joint_state_publisher`
- `xacro`
- `rviz2`
- `tf2`
- `tf2_ros`

## Installation

### 1. Install ROS2 Jazzy Jalisco

Follow the official ROS2 installation instructions:
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
sudo apt update
sudo apt install ros-jazzy-desktop
```

### 2. Install Additional Dependencies

```bash
# Install Python 3.12
sudo apt install python3.12 python3.12-dev python3.12-venv

# Install ROS2 dependencies
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher ros-jazzy-xacro ros-jazzy-rviz2
```

### 3. Create a ROS2 Workspace

```bash
mkdir -p ~/ros2_workspaces/ros2_ws/src
cd ~/ros2_workspaces/ros2_ws/src
```

### 4. Create the UR3 Package

```bash
mkdir -p ur3_pick_place/meshes/{visual,collision}
mkdir -p ur3_pick_place/launch
```

### 5. Download UR3 Mesh Files

```bash
# Create directories for meshes
mkdir -p ~/ros2_workspaces/ros2_ws/src/ur3_pick_place/meshes/visual
mkdir -p ~/ros2_workspaces/ros2_ws/src/ur3_pick_place/meshes/collision

# Download mesh files (example - you may need to find the actual UR3 mesh files)
cd ~/ros2_workspaces/ros2_ws/src/ur3_pick_place/meshes
git clone https://github.com/ros-industrial/universal_robot.git temp
cp -r temp/ur_description/meshes/ur3/visual/* visual/
cp -r temp/ur_description/meshes/ur3/collision/* collision/
rm -rf temp
```

## Project Setup

### 1. Create the URDF File

Create a file named `ur3_absolute_paths_direct.urdf.xacro` in your workspace:

```bash
cd ~/ros2_workspaces
nano ur3_absolute_paths_direct.urdf.xacro
```

Paste the URDF content (see the project files).

### 2. Create the Robot Mover Script

Create a file named `smooth_robot_mover.py` in your workspace:

```bash
cd ~/ros2_workspaces
nano smooth_robot_mover.py
```

Paste the Python script content (see the project files).

Make the script executable:

```bash
chmod +x smooth_robot_mover.py
```

### 3. Create a Launch Script

Create a file named `launch_ur3_mesh_visualization.sh` in your workspace:

```bash
cd ~/ros2_workspaces
nano launch_ur3_mesh_visualization.sh
```

Paste the following content:

```bash
#!/bin/bash

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Kill any existing processes
pkill -f "rviz2" || true
pkill -f "robot_state_publisher" || true
pkill -f "smooth_robot_mover.py" || true

# Wait for processes to terminate
sleep 2

# Start the robot state publisher with the mesh URDF
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ur3_absolute_paths_direct.urdf.xacro)" &
ROBOT_STATE_PUBLISHER_PID=$!

# Wait for the robot state publisher to start
sleep 3

# Start the robot mover
python3.12 smooth_robot_mover.py &
ROBOT_MOVER_PID=$!

# Wait for the robot mover to start
sleep 3

# Start RViz2
rviz2 -d simple_config.rviz &
RVIZ_PID=$!

# Wait for RViz2 to start
sleep 3

echo "All components started successfully!"
echo "Robot State Publisher PID: $ROBOT_STATE_PUBLISHER_PID"
echo "Robot Mover PID: $ROBOT_MOVER_PID"
echo "RViz2 PID: $RVIZ_PID"

# Wait for user to press Ctrl+C
echo "Press Ctrl+C to stop all components"
wait $RVIZ_PID

# Kill all components
kill $ROBOT_STATE_PUBLISHER_PID $ROBOT_MOVER_PID $RVIZ_PID || true
```

Make the script executable:

```bash
chmod +x launch_ur3_mesh_visualization.sh
```

### 4. Create an RViz Configuration

Create a file named `simple_config.rviz` in your workspace:

```bash
cd ~/ros2_workspaces
nano simple_config.rviz
```

You can generate this file by running RViz once and saving the configuration, or use a basic configuration that includes:
- Global Options: Fixed Frame set to "world"
- RobotModel display with Description Topic set to "/robot_description"
- TF display

## Running the Simulation

### Method 1: Using the Launch Script

```bash
cd ~/ros2_workspaces
./launch_ur3_mesh_visualization.sh
```

### Method 2: Running Components Individually

Open three terminal windows:

Terminal 1 (Robot State Publisher):
```bash
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ur3_absolute_paths_direct.urdf.xacro)"
```

Terminal 2 (Robot Mover):
```bash
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
python3.12 smooth_robot_mover.py
```

Terminal 3 (RViz):
```bash
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
rviz2 -d simple_config.rviz
```

## Common Errors and Solutions

### 1. Mesh Files Not Displaying

**Error**: Robot appears as simple shapes instead of detailed meshes.

**Solutions**:
- Check that the mesh file paths in the URDF are correct
- Ensure the mesh files exist in the specified locations
- Try using absolute file paths with the `file://` prefix
- Check RViz settings: ensure "Visual Enabled" is checked in the RobotModel display

### 2. Glitchy Robot Movement

**Error**: Robot movement is jerky or has sudden jumps.

**Solutions**:
- Increase the publishing rate in the robot mover script
- Increase the number of interpolation points between waypoints
- Use a smoother interpolation function (e.g., minimum jerk trajectory)
- Add thread safety with locks
- Ensure consistent timing in the control loop
- Handle angle wrapping properly for revolute joints

### 3. RViz Crashes

**Error**: RViz crashes when trying to display the robot.

**Solutions**:
- Check for errors in the URDF file
- Ensure the mesh files are in the correct format
- Reduce the complexity of the mesh files
- Update your graphics drivers
- Increase the memory available to RViz

### 4. Robot State Publisher Errors

**Error**: Robot state publisher fails to start or publish transforms.

**Solutions**:
- Check the URDF for syntax errors
- Ensure all joint names in the URDF match those used in the joint states message
- Check that the robot_description parameter is being set correctly
- Verify that the TF tree is complete and connected

### 5. Python Script Errors

**Error**: The robot mover script fails to run or has runtime errors.

**Solutions**:
- Check Python version compatibility (script requires Python 3.12)
- Ensure all required Python packages are installed
- Check for syntax errors in the script
- Verify that the joint names match those in the URDF
- Check the ROS2 node initialization and shutdown procedures

## Customizing the Simulation

### 1. Modifying the Robot's Motion

To change the robot's motion pattern, edit the `key_poses` list in the `smooth_robot_mover.py` file:

```python
self.key_poses = [
    # Home position
    [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
    
    # Add or modify poses here
    # Format: [joint1, joint2, joint3, joint4, joint5, joint6]
]
```

### 2. Adjusting Motion Parameters

To make the motion faster, slower, or smoother, adjust these parameters in the script:

```python
self.motion_duration = 8.0  # seconds to move between poses
self.dwell_time = 2.0  # seconds to pause at each pose
```

### 3. Using Different Mesh Files

To use different mesh files, update the file paths in the URDF file:

```xml
<mesh filename="file:///path/to/your/mesh/file.dae"/>
```

## Advanced Usage

### 1. Adding Collision Detection

To add collision detection, you can integrate the simulation with MoveIt:

```bash
sudo apt install ros-jazzy-moveit
```

### 2. Recording the Simulation

To record the simulation as a video:

```bash
# Install required packages
sudo apt install ffmpeg

# Record the screen
ffmpeg -f x11grab -s 1920x1080 -i :0.0 -r 30 -c:v libx264 -preset ultrafast -crf 0 output.mp4
```

### 3. Exporting Joint Trajectories

To export the joint trajectories for use in a real robot:

```bash
# Record joint states to a bag file
ros2 bag record /joint_states
```

## Troubleshooting

### 1. Check ROS2 Environment

```bash
printenv | grep ROS
```

### 2. Check Running ROS2 Nodes

```bash
ros2 node list
```

### 3. Check Published Topics

```bash
ros2 topic list
ros2 topic echo /joint_states
```

### 4. Check TF Tree

```bash
ros2 run tf2_tools view_frames
```

### 5. Check URDF Validity

```bash
check_urdf /tmp/robot_description.urdf
```

## Conclusion

This guide provides comprehensive instructions for setting up and running the UR3 robot pick and place simulation. By following these steps, you should be able to visualize a smooth, glitch-free robot motion with proper 3D mesh models in RViz.

If you encounter any issues not covered in this guide, please check the ROS2 documentation or community forums for additional assistance.
