# Wall Following Multi-Robot System

A ROS2-based multi-robot system featuring a leader-follower behavior with obstacle avoidance and color detection capabilities.

## Overview

This package implements a two-robot system where:
- **Robot1 (Leader)**: Blue robot that follows walls and waits for Robot2 when it falls behind
- **Robot2 (Follower)**: Green robot with sinusoidal motion pattern that follows Robot1

## Features

- **Wall Following**: Both robots avoid obstacles using distance sensors
- **Leader-Follower Behavior**: Robot1 detects Robot2 using color detection and adjusts speed accordingly
- **Sinusoidal Motion**: Robot2 has variable speed (accelerating/decelerating) while maintaining obstacle avoidance
- **Color Detection**: Robot1 uses a rear-facing camera to detect Robot2's green color
- **Multi-Robot Support**: Proper ROS2 namespacing for simultaneous robot control

## Prerequisites

- ROS2 (tested with Kilted)
- Webots simulator
- Python packages: `opencv-python`, `cv-bridge`, `numpy`

## Installation

1. **Navigate to the package directory:**
   ```bash
   cd /home/zaka/ros2_ws/src/wall_following
   ```

2. **Source ROS2 environment:**
   ```bash
   source /opt/ros/kilted/setup.bash
   ```

3. **Source the local workspace:**
   ```bash
   source install/local_setup.bash
   ```

4. **Install dependencies:**
   ```bash
   colcon build --packages-select wall_following
   ```

## Usage

### Running the Multi-Robot System

```bash
# From the wall_following package directory
ros2 launch wall_following multi_robot_launch.py
```

This will:
- Launch Webots simulator with the arena world
- Start Robot1 (leader) with color detection and leader-follower behavior
- Start Robot2 (follower) with sinusoidal motion and obstacle avoidance


## Trajectory Tracking

The system includes comprehensive trajectory tracking capabilities:

### Features
- **Real-time Position Estimation**: Uses velocity integration (dead reckoning) to track robot positions
- **Path Visualization**: Live trajectory visualization in RViz
- **Data Logging**: Automatic logging of trajectory data to JSON files
- **Analysis Tools**: Python scripts for trajectory analysis and visualization

### Usage

#### 1. **Real-time Visualization with RViz**
```bash
# Launch the multi-robot system with trajectory tracking
ros2 launch wall_following multi_robot_launch.py

# In another terminal, launch RViz with trajectory visualization
rviz2 -d /path/to/wall_following/rviz/trajectory_visualization.rviz
```

#### 2. **Trajectory Analysis**
```bash
# Install visualization dependencies
pip install matplotlib numpy

# Analyze trajectories after running the system
cd /home/zaka/ros2_ws/src/new/wall_following
python3 scripts/visualize_trajectories.py --auto

# Generate velocity analysis plots
python3 scripts/visualize_trajectories.py --auto --analysis

# Save plots without displaying
python3 scripts/visualize_trajectories.py --auto --save /tmp/my_trajectory_plot.png --no-show
```

### Trajectory Data

#### **Topics Published**
- `/robot1/robot_path` - Robot1's trajectory path (nav_msgs/Path)
- `/robot1/robot_pose` - Robot1's current pose (geometry_msgs/PoseStamped)
- `/robot2/robot_path` - Robot2's trajectory path (nav_msgs/Path)
- `/robot2/robot_pose` - Robot2's current pose (geometry_msgs/PoseStamped)

#### **Log Files**
Trajectory data is automatically saved to `/tmp/robot_trajectory_YYYYMMDD_HHMMSS.json` with:
- Timestamp
- X, Y coordinates
- Orientation (theta)
- Pose count

#### **Analysis Features**
- Path length calculation
- Average speed analysis
- Velocity profiles over time
- Bounding box analysis
- Duration statistics

### Configuration

#### **Trajectory Tracker Parameters**
```python
# In trajectory_tracker.py
self.__wheel_radius = 0.025  # Wheel radius (meters)
self.__half_distance_between_wheels = 0.045  # Half wheelbase (meters)
```

#### **RViz Configuration**
The RViz configuration shows:
- Grid for reference
- Robot1 path (green)
- Robot2 path (red)
- Real-time pose updates

### Limitations

**Dead Reckoning Accuracy**: Since the robots don't have wheel encoders, position estimation relies on integrating velocity commands. This can accumulate errors over time, especially with:
- Slippage
- Uneven surfaces
- Motor inaccuracies

**Improvement Suggestions**:
1. Add wheel encoders to the robot model
2. Implement sensor fusion with distance sensors
3. Use external positioning systems (camera-based tracking)

## System Architecture

### Robot1 (Leader) - `leader_follower.py`
- **Sensors**: Left/right distance sensors, rear distance sensor, rear camera
- **Behavior**: 
  - Follows walls using obstacle avoidance
  - Detects Robot2 using color detection (HSV range: 40-80° hue)
  - Slows down when Robot2 is too far behind
  - Resumes normal speed when Robot2 catches up

### Robot2 (Follower) - `sinusoidal_motion.py`
- **Sensors**: Left/right distance sensors
- **Behavior**:
  - Sinusoidal speed pattern (base: 0.05, amplitude: 0.05, frequency: 0.5 Hz)
  - Always maintains positive speed (no reverse)
  - Obstacle avoidance with turning behavior

### Robot Driver - `my_robot_driver.py`
- **Function**: Converts ROS2 `cmd_vel` messages to Webots motor commands
- **Features**: Proper namespacing for multi-robot support

## Configuration

### Color Detection Parameters
The system uses HSV color detection with the following range:
- **Hue**: 40-80° (standard green range)
- **Saturation**: 40-255
- **Value**: 40-255
- **Detection Threshold**: 0.6% of image pixels

### Leader-Follower Parameters
- **Base Speed**: 0.12 m/s (Robot1 normal speed)
- **Wait Speed**: 0.01 m/s (Robot1 when waiting)
- **Follow Distance Threshold**: 0.4m (distance to trigger waiting)

### Sinusoidal Motion Parameters
- **Base Speed**: 0.05 m/s
- **Speed Amplitude**: 0.05 m/s
- **Frequency**: 0.5 Hz
- **Avoidance Angular Speed**: -2.0 rad/s

## File Structure

```
wall_following/
├── launch/
│   ├── multi_robot_launch.py    # Multi-robot launch file
│   └── robot_launch.py          # Single robot launch file
├── wall_following/
│   ├── leader_follower.py       # Robot1 behavior node
│   ├── sinusoidal_motion.py     # Robot2 behavior node
│   ├── obstacle_avoider.py     # Basic obstacle avoidance
│   ├── trajectory_tracker.py     # Trajectory tracking node
│   └── my_robot_driver.py      # Robot driver
├── scripts/
│   └── visualize_trajectories.py # Trajectory analysis tool
├── rviz/
│   └── trajectory_visualization.rviz # RViz configuration
├── resource/
│   └── my_robot.urdf           # Robot description
├── worlds/
│   └── my_world.wbt            # Webots world file
├── setup.py                    # Package configuration
└── README.md                   # This file
```

## ROS2 Topics

### Robot1 (robot1 namespace)
- `/robot1/cmd_vel` - Velocity commands
- `/robot1/ds0` - Left distance sensor
- `/robot1/ds1` - Right distance sensor
- `/robot1/back_sensor` - Rear distance sensor
- `/robot1/color_sensor/image_color` - Rear camera

### Robot2 (robot2 namespace)
- `/robot2/cmd_vel` - Velocity commands
- `/robot2/ds0` - Left distance sensor
- `/robot2/ds1` - Right distance sensor

## Debugging

### Color Detection Debugging
The system saves debug images to `/tmp/`:
- `camera_image_X.png` - Raw camera images
- `green_mask_X.png` - Green detection masks

### Logging
- **Periodic Status**: Every 2 seconds showing sensor values and detection status
- **State Changes**: Immediate logging when leader-follower state changes
- **Obstacle Avoidance**: Logs when robots detect and avoid obstacles

## Troubleshooting

### Common Issues

1. **Robots not moving**:
   - Check if ROS2 topics are being published: `ros2 topic list`
   - Verify robot names match in world file and launch file

2. **Color detection not working**:
   - Check camera topic: `ros2 topic info /robot1/color_sensor/image_color`
   - Verify camera is publishing images: `ros2 topic echo /robot1/color_sensor/image_color --once`

3. **Port conflicts**:
   - Ensure each robot has unique `controllerArgs` in world file (1234, 1235)

### Useful Commands

```bash
# List all topics
ros2 topic list

# Check topic info
ros2 topic info /robot1/cmd_vel

# Monitor robot status
ros2 topic echo /robot1/cmd_vel

# Check node status
ros2 node list
```

## Customization

### Adjusting Color Detection
Modify HSV range in `leader_follower.py`:
```python
lower_green = np.array([40, 40, 40])   # Lower bound
upper_green = np.array([80, 255, 255]) # Upper bound
```

### Changing Motion Parameters
Adjust sinusoidal motion in `sinusoidal_motion.py`:
```python
self.__base_speed = 0.05        # Base speed
self.__speed_amplitude = 0.05   # Speed variation
self.__frequency = 0.5          # Frequency in Hz
```

## License

This project is part of a ROS2 learning exercise and is available for educational purposes.
