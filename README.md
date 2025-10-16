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
│   └── my_robot_driver.py      # Robot driver
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
