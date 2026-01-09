# GCAR - Smart Autonomous Garbage Collection and Recycling Vehicle

A ROS 2 Humble simulation-based robotics project featuring intelligent planning and human-aware interaction for urban waste management.

## Project Overview

GCAR is an autonomous garbage collection robot simulated in Gazebo that integrates:
- Autonomous navigation with LiDAR-based SLAM
- Computer vision-based waste detection and classification
- Robotic arm manipulation for waste collection and sorting
- Intelligent mission planning with learning-based route optimization
- Human-aware interaction and safety behaviors

## Group Members

| Name              | ID           |
|-------------------|--------------|
| Shmuye Ayalneh    | UGR/7284/15  |
| Dame Abera        | UGR/0123/15  |
| Abiy Hailu        | UGR/8730/15  |
| Natnael Eyuel     | UGR/4424/15  |
| Yamlak Ngeash     | UGR/2910/15  |
| Kaku Amsalu       | UGR/3710/15  |

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

### Launch the City World (Gazebo Simulation)

```bash
ros2 launch gcar_simulation world.launch.py
```

This will open Gazebo with a realistic urban environment:

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
| 🟢 Green  | Recycling      | Near office, apartment       |
| 🔵 Blue   | Paper          | Near residential             |
| 🟡 Yellow | Metal          | Near warehouse               |
| 🟤 Brown  | Organic        | Near community center        |

**Charging Station:** Located at (-24, -8) with green indicator light

## Project Structure

```
gcar_ws/
├── src/
│   ├── gcar_simulation/      # Gazebo world and simulation environment
│   │   ├── launch/           # Launch files
│   │   └── worlds/           # Gazebo world files
│   ├── gcar_description/     # Robot URDF/Xacro models (planned)
│   ├── gcar_navigation/      # Navigation stack configuration (planned)
│   ├── gcar_perception/      # Waste detection and classification (planned)
│   ├── gcar_manipulation/    # Arm control and MoveIt configuration (planned)
│   ├── gcar_planning/        # Mission planning and state machine (planned)
│   └── gcar_bringup/         # Launch files and system integration (planned)
└── README.md
```

## License

This project is developed for academic purposes as part of robotics coursework.
