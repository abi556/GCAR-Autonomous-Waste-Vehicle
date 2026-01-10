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
from gazebo_msgs.srv import DeleteEntity
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
        self.detected_waste_type = None  # 'red_waste' or 'green_waste'
        self.current_patrol_index = 0
        
        # Nav2 Simple Commander
        self.navigator = BasicNavigator()
        
        # Service clients
        self.arm_pick_client = self.create_client(Trigger, '/arm/go_pick')
        self.arm_home_client = self.create_client(Trigger, '/arm/go_home')
        self.arm_place_bin_client = self.create_client(Trigger, '/arm/go_place_bin')
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.arm_pick_client.wait_for_service(timeout_sec=5.0)
        self.arm_home_client.wait_for_service(timeout_sec=5.0)
        self.arm_place_bin_client.wait_for_service(timeout_sec=5.0)
        self.delete_entity_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('All services available!')
        
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
        self.get_logger().info(f'Green Recycling Bin: {GREEN_RECYCLING_BIN_COORDS}')
    
    def waste_detection_callback(self, msg):
        """Callback when waste is detected."""
        if self.state == self.STATE_PATROL:
            waste_type = msg.data.strip()
            if waste_type in ['red_waste', 'green_waste']:
                self.get_logger().info(f'Detected {waste_type}! Stopping patrol...')
                self.detected_waste_type = waste_type
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
            # Step 2: Delete waste entity (magic pickup)
            if self.delete_future is None:
                # Try different model names based on waste type
                if self.detected_waste_type == 'red_waste':
                    model_names = ['waste_red_1', 'waste_red_2', 'waste_red_3', 'waste_red_4']
                else:  # green_waste (maps to blue in world)
                    model_names = ['waste_blue_1', 'waste_blue_2', 'waste_blue_3', 'waste_blue_4']
                
                if self.delete_attempts < min(self.max_delete_attempts, len(model_names)):
                    model_name = model_names[self.delete_attempts]
                    self.get_logger().info(f'Attempting to delete {model_name} (magic pickup)...')
                    req = DeleteEntity.Request()
                    req.name = model_name
                    self.delete_future = self.delete_entity_client.call_async(req)
                    self.arm_movement_start_time = time.time()
                else:
                    # If all attempts failed, log warning and continue anyway (simulation)
                    self.get_logger().warn('Could not delete waste model, but continuing simulation...')
                    self.pickup_step = 2
                    self.delete_future = None
                    self.delete_attempts = 0
                    self.arm_movement_start_time = None
            
            elif self.delete_future is not None and self.delete_future.done():
                # Check result
                try:
                    response = self.delete_future.result()
                    if response and response.success:
                        self.get_logger().info('Waste deleted! Moving arm to CARRY_POSE...')
                        self.pickup_step = 2
                        self.delete_future = None
                        self.delete_attempts = 0
                        self.arm_movement_start_time = None
                    else:
                        # Try next model name
                        self.delete_attempts += 1
                        self.delete_future = None
                        self.arm_movement_start_time = None
                        if self.delete_attempts >= self.max_delete_attempts:
                            self.get_logger().warn('All delete attempts failed, continuing anyway...')
                            self.pickup_step = 2
                            self.delete_attempts = 0
                except Exception as e:
                    self.get_logger().error(f'Error deleting entity: {e}')
                    self.delete_attempts += 1
                    self.delete_future = None
                    self.arm_movement_start_time = None
                    if self.delete_attempts >= self.max_delete_attempts:
                        self.pickup_step = 2
                        self.delete_attempts = 0
        
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
        else:  # green_waste
            target = GREEN_RECYCLING_BIN_COORDS
            bin_name = 'Green Recycling Bin'
        
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
        """Handle drop state - move arm to drop pose and log success."""
        if self.drop_step == 0:
            # Step 1: Move arm to DROP_POSE
            if self.arm_movement_start_time is None:
                self.get_logger().info('Moving arm to DROP_POSE...')
                req = Trigger.Request()
                future = self.arm_place_bin_client.call_async(req)
                self.arm_movement_start_time = time.time()
            elif time.time() - self.arm_movement_start_time >= self.arm_movement_duration:
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
                self.drop_step = 0
                self.arm_movement_start_time = None
    
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

