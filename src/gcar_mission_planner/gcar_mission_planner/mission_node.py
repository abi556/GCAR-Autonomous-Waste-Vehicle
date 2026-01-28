#!/usr/bin/env python3
"""Main mission planner node with state machine for waste pickup and transport.

This node implements a complete waste collection mission:
1. Patrol: Navigate to random points
2. Detection: When waste is detected, stop and pick it up
3. Pickup: Move arm to pick pose, delete waste entity, move arm to carry pose
4. Transport: Navigate to appropriate bin (red -> red dumpster, green -> green recycling)
5. Drop: Move arm to drop pose, log success
6. Resume: Return to patrol
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander import BasicNavigator
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
import random
import time
import math


# Bin locations (global variables as requested)
RED_DUMPSTER_COORDS = (3.5, 8.0)  # Red bin location
GREEN_RECYCLING_BIN_COORDS = (3.5, 12.0)  # Green/Blue recycling bin location

# Patrol waypoints (random points for patrolling)
PATROL_WAYPOINTS = [
    (0.0, 0.0),
    (2.0, 2.0),
    (-2.0, 2.0),
    (2.0, -2.0),
    (-2.0, -2.0),
    (5.0, 0.0),
    (-5.0, 0.0),
    (0.0, 5.0),
    (0.0, -5.0),
]


class MissionPlanner(Node):
    """Main mission planner with state machine for waste collection."""
    
    # State definitions
    STATE_PATROL = 'PATROL'
    STATE_DETECTED = 'DETECTED'
    STATE_PICKUP = 'PICKUP'
    STATE_TRANSPORT = 'TRANSPORT'
    STATE_DROP = 'DROP'
    
    def __init__(self):
        super().__init__('mission_planner')
        
        # State machine
        self.state = self.STATE_PATROL
        self.detected_waste_type = None  # 'red_waste' or 'blue_waste'
        self.current_patrol_index = 0
        
        # Nav2 Simple Commander
        self.navigator = BasicNavigator()
        
        # Robot pose (for distance checks to known waste locations)
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Subscribe to odometry so we can compute distance to known waste
        self.odom_sub = self.create_subscription(
            Odometry,
            '/gcar/odom',
            self.odom_callback,
            10,
        )

        # Service clients
        self.arm_pick_client = self.create_client(Trigger, '/arm/go_pick')
        self.arm_home_client = self.create_client(Trigger, '/arm/go_home')
        self.arm_place_bin_client = self.create_client(Trigger, '/arm/go_place_bin')
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.arm_pick_client.wait_for_service(timeout_sec=5.0)
        self.arm_home_client.wait_for_service(timeout_sec=5.0)
        self.arm_place_bin_client.wait_for_service(timeout_sec=5.0)
        self.delete_entity_client.wait_for_service(timeout_sec=5.0)
        self.spawn_entity_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('All services available!')

        # --- Waste tracking list (8 cubes in the world) ---
        # These coordinates are taken from city.world. We treat this list as
        # the ground-truth "catalog" of waste that exists in the mission.
        # color: 'red' or 'blue' (blue replaces previous "green" recyclable)
        self.waste_items = [
            {'name': 'waste_red_1',  'color': 'red',  'x':  1.5, 'y':  1.0, 'picked': False},
            {'name': 'waste_red_2',  'color': 'red',  'x':  1.0, 'y': -3.0, 'picked': False},
            {'name': 'waste_red_3',  'color': 'red',  'x':  5.0, 'y':  1.0, 'picked': False},
            {'name': 'waste_red_4',  'color': 'red',  'x':  3.0, 'y': -4.0, 'picked': False},
            {'name': 'waste_blue_1', 'color': 'blue', 'x': -1.5, 'y':  1.0, 'picked': False},
            {'name': 'waste_blue_2', 'color': 'blue', 'x': -1.0, 'y':  4.0, 'picked': False},
            {'name': 'waste_blue_3', 'color': 'blue', 'x': -5.0, 'y': -1.0, 'picked': False},
            {'name': 'waste_blue_4', 'color': 'blue', 'x': -3.0, 'y': -4.0, 'picked': False},
        ]

        # Track which specific waste this mission cycle is targeting
        self.current_target_waste = None  # dict from waste_items

        # Simple world-count guard: never allow more than the original 8
        self.max_world_waste_count = len(self.waste_items)
        self.current_world_waste_count = len(self.waste_items)
        
        # Subscriber for waste detection
        self.waste_sub = self.create_subscription(
            String,
            '/detected_waste',
            self.waste_detection_callback,
            10
        )
        
        # State machine timer (runs at 1 Hz)
        self.state_timer = self.create_timer(1.0, self.state_machine_step)
        
        # Arm movement tracking
        self.arm_movement_start_time = None
        self.arm_movement_duration = 2.0  # Wait 2 seconds between arm movements
        
        # Pickup tracking
        self.pickup_step = 0  # 0: move to pick, 1: delete entity, 2: move to carry
        self.delete_future = None
        self.delete_attempts = 0
        self.max_delete_attempts = 3  # Try up to 3 model names
        
        # Drop tracking
        self.drop_step = 0  # 0: move to drop, 1: log success
        
        self.get_logger().info('Mission Planner Node Started')
        self.get_logger().info(f'Initial State: {self.state}')
        self.get_logger().info(f'Red Dumpster: {RED_DUMPSTER_COORDS}')
        self.get_logger().info(f'Blue Recycling Bin: {GREEN_RECYCLING_BIN_COORDS}')

    # ------------------------------------------------------------------
    # Core callbacks / helpers
    # ------------------------------------------------------------------

    def odom_callback(self, msg: Odometry) -> None:
        """Update robot position from odometry for distance checks."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
    
    def waste_detection_callback(self, msg):
        """Callback when waste is detected.

        We apply an "Already Picked" filter using the known waste list:
        - Determine the nearest AVAILABLE waste of the matching color.
        - If it is farther than 1.0 m from the robot, we ignore the
          detection as a ghost / stale perception.
        """
        if self.state != self.STATE_PATROL:
            return

        waste_type = msg.data.strip()
        if waste_type not in ['red_waste', 'blue_waste']:
            return

        color = 'red' if waste_type == 'red_waste' else 'blue'

        nearest_waste, distance = self._get_nearest_available_waste(color)
        if nearest_waste is None:
            self.get_logger().info(
                f'Ignoring {waste_type} detection: no available {color} wastes left.'
            )
            return

        if distance is None or distance > 1.0:
            self.get_logger().info(
                f'Ignoring {waste_type} detection: nearest known {color} waste is '
                f'{distance:.2f if distance is not None else float("nan"):.2f} m away (> 1.0 m).'
            )
            return

        # Detection is spatially consistent with a known, not-yet-picked waste
        self.current_target_waste = nearest_waste
        self.detected_waste_type = waste_type
        self.get_logger().info(
            f'Accepted {waste_type} detection. Targeting waste '
            f'{nearest_waste["name"]} at ({nearest_waste["x"]:.2f}, {nearest_waste["y"]:.2f}), '
            f'distance {distance:.2f} m.'
        )
        self.state = self.STATE_DETECTED
        # Stop the robot by canceling current navigation
        self.navigator.cancelTask()
    
    def state_machine_step(self):
        """Main state machine loop."""
        if self.state == self.STATE_PATROL:
            self._handle_patrol()
        elif self.state == self.STATE_DETECTED:
            self._handle_detected()
        elif self.state == self.STATE_PICKUP:
            self._handle_pickup()
        elif self.state == self.STATE_TRANSPORT:
            self._handle_transport()
        elif self.state == self.STATE_DROP:
            self._handle_drop()
    
    def _handle_patrol(self):
        """Handle patrol state - navigate to random waypoints."""
        if not self.navigator.isTaskComplete():
            return  # Still navigating
        
        # Select next patrol waypoint
        waypoint = random.choice(PATROL_WAYPOINTS)
        self.get_logger().info(f'Patrolling to {waypoint}')
        self.navigator.goToPose(self._create_pose(waypoint[0], waypoint[1], 0.0))
    
    def _handle_detected(self):
        """Handle detected state - transition to pickup."""
        self.get_logger().info('Waste detected! Starting pickup sequence...')
        self.state = self.STATE_PICKUP
        self.pickup_step = 0
        self.arm_movement_start_time = None
        self.delete_future = None
        self.delete_attempts = 0
    
    def _handle_pickup(self):
        """Handle pickup state - move arm, delete entity, move arm to carry."""
        if self.pickup_step == 0:
            # Step 1: Move arm to PICK_POSE
            if self.arm_movement_start_time is None:
                self.get_logger().info('Moving arm to PICK_POSE...')
                req = Trigger.Request()
                future = self.arm_pick_client.call_async(req)
                self.arm_movement_start_time = time.time()
            elif time.time() - self.arm_movement_start_time >= self.arm_movement_duration:
                self.get_logger().info('Arm at PICK_POSE. Deleting waste entity...')
                self.pickup_step = 1
                self.arm_movement_start_time = None
        
        elif self.pickup_step == 1:
            # Step 2: Delete SPECIFIC waste entity from the tracking list
            if self.current_target_waste is None:
                self.get_logger().warn(
                    'Pickup step requested but no current_target_waste set; '
                    'skipping delete and continuing.'
                )
                self.pickup_step = 2
                return

            if self.delete_future is None:
                model_name = self.current_target_waste['name']
                self.get_logger().info(
                    f'Deleting tracked waste entity {model_name} (magic pickup)...'
                )
                req = DeleteEntity.Request()
                req.name = model_name
                self.delete_future = self.delete_entity_client.call_async(req)
                self.arm_movement_start_time = time.time()

            elif self.delete_future.done():
                try:
                    response = self.delete_future.result()
                    if response and response.success:
                        self.get_logger().info(
                            f'Waste {self.current_target_waste["name"]} deleted from Gazebo.'
                        )
                    else:
                        self.get_logger().warn(
                            f'Gazebo delete for {self.current_target_waste["name"]} failed '
                            '(or returned no success). Treating as picked for mission logic.'
                        )
                except Exception as e:
                    self.get_logger().error(
                        f'Error deleting entity {self.current_target_waste["name"]}: {e}'
                    )

                # In all cases, mark this waste as picked and update count
                self._mark_waste_picked(self.current_target_waste)
                self.get_logger().info('Waste picked. Moving arm to CARRY_POSE...')
                self.pickup_step = 2
                self.delete_future = None
                self.arm_movement_start_time = None
        
        elif self.pickup_step == 2:
            # Step 3: Move arm to CARRY_POSE (home position)
            if self.arm_movement_start_time is None:
                req = Trigger.Request()
                future = self.arm_home_client.call_async(req)
                self.arm_movement_start_time = time.time()
            elif time.time() - self.arm_movement_start_time >= self.arm_movement_duration:
                self.get_logger().info('Arm at CARRY_POSE. Starting transport...')
                self.state = self.STATE_TRANSPORT
                self.pickup_step = 0
                self.arm_movement_start_time = None
    
    def _handle_transport(self):
        """Handle transport state - navigate to appropriate bin."""
        if not self.navigator.isTaskComplete():
            return  # Still navigating
        
        # Determine target bin based on waste type
        if self.detected_waste_type == 'red_waste':
            target = RED_DUMPSTER_COORDS
            bin_name = 'Red Dumpster'
        else:  # blue_waste
            target = GREEN_RECYCLING_BIN_COORDS
            bin_name = 'Blue Recycling Bin'
        
        self.get_logger().info(f'Transporting to {bin_name} at {target}...')
        self.navigator.goToPose(self._create_pose(target[0], target[1], 0.0))
        
        # Wait a bit, then check if we've arrived
        if not hasattr(self, '_transport_start_time'):
            self._transport_start_time = time.time()
        elif time.time() - self._transport_start_time > 1.0:
            # Check if navigation is complete
            if self.navigator.isTaskComplete():
                self.get_logger().info(f'Arrived at {bin_name}!')
                self.state = self.STATE_DROP
                self.drop_step = 0
                self.arm_movement_start_time = None
                delattr(self, '_transport_start_time')
    
    def _handle_drop(self):
        """Handle drop state - move arm to drop pose, spawn waste at bin, and log success."""
        if self.drop_step == 0:
            # Step 1: Move arm to DROP_POSE
            if self.arm_movement_start_time is None:
                self.get_logger().info('Moving arm to DROP_POSE...')
                req = Trigger.Request()
                future = self.arm_place_bin_client.call_async(req)
                self.arm_movement_start_time = time.time()
            elif time.time() - self.arm_movement_start_time >= self.arm_movement_duration:
                # Step 1 complete – now actually "place" the specific waste at the bin.
                if self.current_target_waste is not None:
                    self._spawn_waste_at_bin(self.current_target_waste)
                else:
                    self.get_logger().warn(
                        'Drop step reached but current_target_waste is None; '
                        'skipping spawn.'
                    )
                self.get_logger().info('Waste successfully recycled.')
                self.drop_step = 1
                self.arm_movement_start_time = None
        
        elif self.drop_step == 1:
            # Step 2: Return arm to home and resume patrol
            if self.arm_movement_start_time is None:
                req = Trigger.Request()
                future = self.arm_home_client.call_async(req)
                self.arm_movement_start_time = time.time()
            elif time.time() - self.arm_movement_start_time >= self.arm_movement_duration:
                self.get_logger().info('Resuming patrol...')
                self.state = self.STATE_PATROL
                self.detected_waste_type = None
                self.current_target_waste = None
                self.drop_step = 0
                self.arm_movement_start_time = None

    # ------------------------------------------------------------------
    # Waste tracking helpers
    # ------------------------------------------------------------------

    def _get_nearest_available_waste(self, color):
        """Return (waste_dict, distance) for nearest NOT picked waste of given color."""
        nearest = None
        nearest_dist = None
        for item in self.waste_items:
            if item['picked']:
                continue
            if item['color'] != color:
                continue
            dx = item['x'] - self.robot_x
            dy = item['y'] - self.robot_y
            dist = math.sqrt(dx * dx + dy * dy)
            if nearest is None or dist < nearest_dist:
                nearest = item
                nearest_dist = dist
        return nearest, nearest_dist

    def _mark_waste_picked(self, item):
        """Mark a waste item as picked and update world count."""
        if not item['picked']:
            item['picked'] = True
            self.current_world_waste_count = max(0, self.current_world_waste_count - 1)
            self.get_logger().info(
                f'Marked {item["name"]} as PICKED. '
                f'World waste count: {self.current_world_waste_count}/{self.max_world_waste_count}'
            )

    def _mark_waste_dropped(self, item):
        """Mark a waste item as present in world again (at bin)."""
        if item['picked']:
            item['picked'] = False
            self.current_world_waste_count = min(
                self.max_world_waste_count,
                self.current_world_waste_count + 1,
            )
            self.get_logger().info(
                f'Marked {item["name"]} as DROPPED. '
                f'World waste count: {self.current_world_waste_count}/{self.max_world_waste_count}'
            )

    def _can_spawn_more_waste(self):
        """Guard: ensure we never exceed the original number of waste entities."""
        if self.current_world_waste_count >= self.max_world_waste_count:
            self.get_logger().warn(
                'World waste count already at maximum; refusing to spawn more waste.'
            )
            return False
        return True

    def _spawn_waste_at_bin(self, item):
        """Spawn the specific waste cube at the appropriate bin coordinates."""
        if not self._can_spawn_more_waste():
            return

        # Select bin coordinates based on color
        if item['color'] == 'red':
            x, y = RED_DUMPSTER_COORDS
        else:
            x, y = GREEN_RECYCLING_BIN_COORDS

        from geometry_msgs.msg import Pose

        req = SpawnEntity.Request()
        req.name = item['name']
        req.xml = self._generate_waste_sdf(item['color'])
        req.initial_pose = Pose()
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = 0.55  # slightly above bin rim

        self.get_logger().info(
            f'Spawning {item["name"]} ({item["color"]}) at bin ({x:.2f}, {y:.2f})...'
        )
        future = self.spawn_entity_client.call_async(req)
        # We don't block the whole node; just check result later.
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is not None and future.result().success:
            self.get_logger().info(
                f'Spawned {item["name"]} successfully at bin ({x:.2f}, {y:.2f}).'
            )
            self._mark_waste_dropped(item)
        else:
            self.get_logger().warn(
                f'Failed to spawn {item["name"]} at bin (or no response). '
                'Keeping it marked as picked to avoid over-counting.'
            )

    def _generate_waste_sdf(self, color: str) -> str:
        """Generate a simple SDF for a colored waste cube."""
        if color == 'red':
            ambient = "0.9 0.1 0.1 1"
            diffuse = "1.0 0.2 0.2 1"
        else:
            ambient = "0.05 0.15 0.9 1"
            diffuse = "0.10 0.25 1.0 1"

        sdf = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='waste_cube'>
    <static>false</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
        <material>
          <ambient>{ambient}</ambient>
          <diffuse>{diffuse}</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>"""
        return sdf
    
    def _create_pose(self, x, y, yaw):
        """Create a PoseStamped from x, y, yaw."""
        from geometry_msgs.msg import PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        return pose


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = MissionPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.destroyNode()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

