# GCAR - Smart Autonomous Garbage Collection and Recycling Vehicle

A ROS 2 Humble simulation-based robotics project featuring intelligent planning and human-aware interaction for urban waste management.

## Project Overview

GCAR is an autonomous garbage collection robot simulated in Gazebo that integrates:
- Autonomous navigation with LiDAR-based SLAM
- Computer vision-based waste detection and classification
- Robotic arm manipulation for waste collection and sorting
- Intelligent mission planning with learning-based route optimization
- Human-aware interaction and safety behaviors

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Classic (gazebo11)
- Python 3.10+
- colcon build tools

## How to Build

1. **Clone the repository:**
   ```bash
   cd ~/
   git clone <repository-url> gcar_ws
   cd gcar_ws
   ```

2. **Install dependencies:**
   ```bash
   cd ~/gcar_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   cd ~/gcar_ws
   colcon build --symlink-install
   ```

4. **Source the workspace:**
   ```bash
   source ~/gcar_ws/install/setup.bash
   ```

   > **Tip:** Add this line to your `~/.bashrc` for automatic sourcing:
   > ```bash
   > echo "source ~/gcar_ws/install/setup.bash" >> ~/.bashrc
   > ```

## How to Run

### View Robot in RViz2 (without Gazebo)

```bash
ros2 launch gcar_description display.launch.py
```

This opens RViz2 with:
- Robot model visualization
- Joint state publisher GUI to move arm joints
- TF frames display

### Launch Robot in City World (Full Simulation)

```bash
ros2 launch gcar_description spawn_robot.launch.py
```

This will:
1. Launch Gazebo with the city world
2. Spawn the GCAR robot on the main road near the intersection (default: x = -5.0, y = 0.0)
3. Start robot state publisher

### Launch City World Only (without robot)

```bash
ros2 launch gcar_simulation world.launch.py
```

## Testing Guide

### 1. View Robot in RViz2 (URDF only, no Gazebo)

```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash
ros2 launch gcar_description display.launch.py
```

- **What it does**: Opens RViz2 with the GCAR robot model and the Joint State Publisher GUI so you can move the arm joints and verify the URDF and TF frames.

---

### 2. Full Simulation: City World + Robot Spawn

```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash
ros2 launch gcar_description spawn_robot.launch.py
```

- **What it does**: Starts Gazebo with the GCAR city world, spawns the GCAR robot on the main road near the intersection, and runs `robot_state_publisher` so all TF and `robot_description` are available.

---

### 3. Move the Robot with `cmd_vel` (Planar Move Plugin)

First terminal:

```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash
ros2 launch gcar_description spawn_robot.launch.py
```

Second terminal:

```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash
ros2 topic pub /gcar/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

- **What it does**: Publishes a forward velocity command so the robot drives forward using differential drive (wheels spin, robot moves like a tank/car).

To stop the robot:

```bash
ros2 topic pub /gcar/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

- **What it does**: Sends zero velocity so the robot stops moving.

---

### 4. Check LiDAR, Camera, and Odometry Topics

With the simulation running (`spawn_robot.launch.py`):

```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash

# List GCAR-related topics
ros2 topic list | grep gcar

# Check one LiDAR message
ros2 topic echo /gcar/scan --once

# Check one odometry message
ros2 topic echo /gcar/odom --once
```

- **What they do**: Confirm that LiDAR and odometry topics exist and are publishing sane data.

Optional camera visualization:

```bash
ros2 run rqt_image_view rqt_image_view
```

- **What it does**: Opens an image viewer; select `/gcar/camera/image_raw` in the dropdown to see the RGB camera feed (if your Qt/Wayland setup allows).

---

### 5. Troubleshooting Gazebo Already Running

If you see "Address already in use" errors:

```bash
killall -9 gazebo gzserver gzclient
```

- **What it does**: Force-kills any leftover Gazebo processes so you can relaunch cleanly.

## Robot Description

**GCAR Robot Features:**
- **Chassis:** Rectangular box (0.5m × 0.3m × 0.15m) raised above ground on wheels
- **Drive:** Differential drive with two rear drive wheels + front caster wheel
- **Control:** `cmd_vel` via `libgazebo_ros_diff_drive.so` plugin
- **Arm:** 3-DOF articulated arm (base rotation, shoulder pitch, elbow pitch)
- **Gripper:** Vacuum-style gripper at end effector
- **Sensors:**
  - LiDAR (360° scan, 10m range) on `/gcar/scan`
  - RGB Camera (640×480) on `/gcar/camera/image_raw`
  - Depth Camera on `/gcar/depth/image_raw`

**Wheel Configuration:**
- **Rear Wheels:** 2× drive wheels (radius 0.06m) connected to diff_drive
- **Front Wheels:** 2× passive wheels (radius 0.06m) free-spinning
- **Wheel Separation:** 0.34m between left and right wheels

**ROS Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/gcar/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/gcar/odom` | `nav_msgs/Odometry` | Odometry |
| `/gcar/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/gcar/camera/image_raw` | `sensor_msgs/Image` | RGB camera |
| `/gcar/joint_states` | `sensor_msgs/JointState` | Arm joint states |

## City Environment

**Infrastructure:**
- Roads with lane markings (cross intersection)
- Sidewalks in all four quadrants
- Street lamp posts with lights
- Park benches and trees

**Buildings:**
- Office buildings (blue/gray)
- Residential buildings (beige/cream)
- Warehouse (gray)
- Shops and community center

**Smart Bins (Color-coded by waste type):**
| Color  | Type           | Locations                    |
|--------|----------------|------------------------------|
| 🔴 Red    | General Waste  | Near office, residential, warehouse |
| 🔵 Blue   | Recycling      | Near office, apartment, residential |
| 🟡 Yellow | Metal          | Near warehouse               |
| 🟤 Brown  | Organic        | Near community center        |

**Charging Station:** Located at (-24, -8) with green indicator light

## Navigation & SLAM

### Launch Navigation Stack

First, start the simulation with the robot:

```bash
# Terminal 1: Launch simulation
ros2 launch gcar_description spawn_robot.launch.py
```

Then launch navigation with SLAM:

```bash
# Terminal 2: Launch SLAM + Nav2 + RViz
ros2 launch gcar_navigation navigation.launch.py
```

### Build a Map (Teleoperation)

Use WASD keyboard teleop to drive the robot and build a map:

```bash
# Terminal 3: WASD keyboard teleop
ros2 run gcar_navigation teleop_wasd
```

**Controls:**
- `W` - Move forward
- `S` - Move backward
- `A` - Turn left (rotate counter-clockwise)
- `D` - Turn right (rotate clockwise)
- `W+A` - Move forward while turning left
- `W+D` - Move forward while turning right
- `Q` - Stop
- `+` / `-` - Increase/decrease linear speed (10%)
- `[` / `]` - Increase/decrease angular speed (10%)

**Tip:** Hold keys for continuous movement. Adjust speeds with `+/-` and `[/]` if rotation feels too fast/slow.

### Save the Map

After building a map, save it:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/Rob_proj/gcar_ws/maps/city_map
```

### Autonomous Navigation

1. In RViz, use **"2D Pose Estimate"** to set the robot's initial pose
2. Use **"Nav2 Goal"** button to set navigation goals
3. The robot will autonomously plan and navigate to the goal

### Navigation Features

- **SLAM Toolbox**: Online async mapping using LiDAR
- **Nav2 Planner**: NavFn global path planning
- **DWB Controller**: Differential drive local planning
- **Costmaps**: Obstacle avoidance with inflation layers
- **Recovery Behaviors**: Spin, backup, wait on stuck

## Perception - Waste Detection

The `gcar_perception` package provides waste detection using color thresholding:

- **Red objects** → Hazardous/General Waste
- **Blue objects** → Recyclable

### Run Waste Detector

With the simulation running:

```bash
# Terminal: Start waste detection
ros2 run gcar_perception waste_detector
```

The node subscribes to `/gcar/camera/image_raw` and publishes detected waste to `/detected_waste`.

### Verify Camera Stream in RViz

1. Start the simulation: `ros2 launch gcar_description spawn_robot.launch.py`
2. Open RViz2: `rviz2`
3. Click **Add** → **By topic** → Select `/gcar/camera/image_raw` → **Image**
4. You should see the robot's camera feed

**Alternative - Using rqt_image_view:**

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/gcar/camera/image_raw` from the dropdown.

### Test Waste Detection

```bash
# Monitor detected waste
ros2 topic echo /detected_waste
```

Place red or blue bins in front of the robot's camera to trigger detections.

### Detection Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `min_contour_area` | 2500 px² | Minimum object size to detect |
| `min_image_coverage` | 1.5% | Minimum percentage of image (ensures close proximity) |
| `detection_cooldown` | 1.0 s | Time between consecutive detections |

**HSV Color Ranges (OpenCV HSV):**
- Red: H(0-10, 160-180), S(100-255), V(100-255)
- Blue: H(95-135), S(80-255), V(60-255)

## Safety & Boundary Monitoring

The `gcar_safety` package provides operational boundary enforcement to keep the robot within safe limits.

### Boundary Monitor

The boundary monitor tracks the robot's position and prevents it from leaving the operational area (city boundaries).

**Features:**
- Monitors robot position from `/gcar/odom`
- Warns when approaching boundaries (within 5m)
- Emergency stop when out of bounds (optional)
- Configurable boundary limits

**Default Operational Area:**
- X: -32m to +32m
- Y: -32m to +32m
- Total: 64m × 64m (covers entire city)

### Run Boundary Monitor

With the simulation running:

```bash
# Terminal: Start boundary monitoring
ros2 run gcar_safety boundary_monitor
```

The node will:
1. Monitor robot position continuously
2. Log warnings when approaching boundaries
3. Publish warnings to `/gcar/safety/boundary_warning`
4. Issue emergency stop if robot exceeds boundaries (default: enabled)

### Monitor Boundary Warnings

```bash
# Check boundary warnings
ros2 topic echo /gcar/safety/boundary_warning
```

### Configure Boundaries

You can adjust boundaries by setting parameters:

```bash
ros2 run gcar_safety boundary_monitor \
  --ros-args \
  -p x_min:=-40.0 \
  -p x_max:=40.0 \
  -p y_min:=-40.0 \
  -p y_max:=40.0 \
  -p warning_margin:=8.0 \
  -p emergency_stop:=true
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `x_min` | -32.0 | Minimum X coordinate (meters) |
| `x_max` | 32.0 | Maximum X coordinate (meters) |
| `y_min` | -32.0 | Minimum Y coordinate (meters) |
| `y_max` | 32.0 | Maximum Y coordinate (meters) |
| `warning_margin` | 5.0 | Distance from boundary to start warning (meters) |
| `emergency_stop` | true | Stop robot if out of bounds |

## Project Structure

```
gcar_ws/
├── src/
│   ├── gcar_description/     # Robot URDF/Xacro models
│   │   ├── urdf/             # Robot description files
│   │   ├── launch/           # Display and spawn launch files
│   │   └── rviz/             # RViz configuration
│   ├── gcar_simulation/      # Gazebo world and simulation
│   │   ├── launch/           # World launch files
│   │   └── worlds/           # Gazebo world files
│   ├── gcar_navigation/      # SLAM and Nav2 configuration
│   │   ├── config/           # SLAM Toolbox params
│   │   ├── params/           # Nav2 params
│   │   ├── launch/           # Navigation launch files
│   │   └── rviz/             # Nav2 RViz config
│   ├── gcar_perception/      # Waste detection using color thresholding
│   │   └── gcar_perception/  # Python nodes (waste_detector.py)
│   ├── gcar_safety/          # Boundary monitoring and safety systems
│   │   └── gcar_safety/      # Python nodes (boundary_monitor.py)
│   ├── gcar_manipulation/    # Arm control (planned)
│   ├── gcar_planning/        # Mission planning (planned)
│   └── gcar_bringup/         # System integration (planned)
└── README.md
```

## License

This project is developed for academic purposes as part of robotics coursework.
