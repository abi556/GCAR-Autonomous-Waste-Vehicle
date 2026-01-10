# Magic Pickup System - Test Plan

## Architecture Overview

```
waste_detector → pickup_coordinator → arm_controller
                          ↓               ↓
                   gazebo_manager ← (synchronizes)
```

## Test Procedure

### Terminal 1: Gazebo + Robot
```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash
ros2 launch gcar_description spawn_robot.launch.py
```

### Terminal 2: Waste Detector
```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash
ros2 run gcar_perception waste_detector
```

### Terminal 3: Arm Controllers
```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash
ros2 launch gcar_manipulation arm_control_simple.launch.py
```

### Terminal 4: Gazebo Manager
```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash
ros2 run gcar_manipulation gazebo_manager
```

### Terminal 5: Pickup Coordinator
```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash
ros2 run gcar_manipulation pickup_coordinator
```

### Terminal 6: Teleop (for manual positioning)
```bash
cd ~/Rob_proj/gcar_ws
source install/setup.bash
ros2 run gcar_navigation teleop_wasd
```

## Expected Behavior

1. **IDLE**: Coordinator waits for waste detection
2. **Detection**: Drive robot (WASD) near red waste at (1.5, 1.0)
3. **Trigger**: When waste_detector publishes "red_waste"
4. **Workflow**:
   - State: APPROACH_WASTE (2s delay simulation)
   - State: PICKUP (arm lowers, waste deleted from Gazebo)
   - State: NAVIGATE_TO_BIN (2s delay simulation)
   - State: PLACE (arm to bin, waste spawned at bin)
   - State: RETURN_HOME (arm home)
   - State: IDLE (ready for next waste)

## Manual Service Testing

Test individual components:

```bash
# Test arm movements
ros2 service call /arm/go_home std_srvs/srv/Trigger
ros2 service call /arm/go_pick std_srvs/srv/Trigger
ros2 service call /arm/go_place_bin std_srvs/srv/Trigger

# Test Gazebo magic
ros2 service call /gazebo/pickup_waste std_srvs/srv/Trigger
ros2 service call /gazebo/place_waste std_srvs/srv/Trigger

# Monitor detection
ros2 topic echo /detected_waste
```

## Waste Cube Locations

- Red: (1.5, 1.0), (1.0, -3.0), (5.0, 1.0), (3.0, -4.0)
- Blue: (-1.5, 1.0), (-1.0, 4.0), (-5.0, -1.0), (-3.0, -4.0)

## Bin Locations

- Red: (3.5, 8.0), (-3.5, -8.0), (10.0, 3.5)
- Blue: (3.5, 12.0), (-3.5, -12.0), (-10.0, -3.5)

## Known Limitations

1. Navigation is simulated (time delays, not actual driving)
2. Waste selection is hardcoded (waste_red_1)
3. Bin location is hardcoded (3.5, 8.0)
4. Full integration requires position tracking

## Success Criteria

✅ Arm moves to correct poses
✅ Waste disappears when picked (Gazebo delete)
✅ Waste appears at bin when placed (Gazebo spawn)
✅ State machine completes full cycle
✅ Returns to IDLE for next waste
