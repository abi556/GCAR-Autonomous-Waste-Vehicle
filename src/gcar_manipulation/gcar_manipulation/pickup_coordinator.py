#!/usr/bin/env python3
"""Pickup coordinator - simple state machine for waste collection workflow.

States:
1. IDLE - Waiting for waste detection
2. APPROACH_WASTE - Drive toward detected waste
3. PICKUP - Lower arm and delete waste model
4. NAVIGATE_TO_BIN - Drive to matching bin
5. PLACE - Raise arm and spawn waste at bin
6. RETURN_HOME - Return arm to home position

This is a simplified coordinator without SMACH for rapid prototyping.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import math
import time


class PickupCoordinator(Node):
    """Simple state machine coordinator for waste pickup workflow."""
    
    # State definitions
    STATE_IDLE = 'IDLE'
    STATE_APPROACH_WASTE = 'APPROACH_WASTE'
    STATE_ALIGN_TO_WASTE = 'ALIGN_TO_WASTE'  # NEW: Face waste before picking
    STATE_PICKUP = 'PICKUP'
    STATE_NAVIGATE_TO_BIN = 'NAVIGATE_TO_BIN'
    STATE_PLACE = 'PLACE'
    STATE_BACKUP_FROM_BIN = 'BACKUP_FROM_BIN'
    STATE_RETURN_HOME = 'RETURN_HOME'
    
    def __init__(self):
        super().__init__('pickup_coordinator')
        
        # Current state
        self.state = self.STATE_IDLE
        self.detected_waste_type = None  # 'red_waste' or 'blue_waste'
        self.waste_position = None  # (x, y) tuple of detected waste
        self.carrying_waste_type = None  # 'red' or 'blue' when carrying waste (persists through aborts)
        
        # Robot position and orientation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Navigation state tracking
        self.navigation_target = None  # (x, y) tuple when navigating
        self.last_nav_log_time = 0.0
        
        # Pickup verification
        self.pickup_success = False
        
        # Bin proximity check timer (for automatic drop when manually approaching bin)
        self.last_bin_check_time = 0.0
        self.bin_check_interval = 1.5  # seconds between checks
        
        # Publisher for cmd_vel (for alignment rotation)
        self.cmd_vel_pub = self.create_publisher(Twist, '/gcar/cmd_vel', 10)
        
        # Waste and bin locations (hardcoded for simplicity)
        self.waste_locations = {
            'red': (1.5, 1.0),
            'blue': (-1.5, 1.0),
        }
        # Bin locations: multiple roadside bins of each color.
        # We will choose the NEAREST bin at runtime.
        self.bin_locations = {
            'red': [
                (3.5, 8.0),     # roadside_bin_red_1
                (-3.5, -8.0),   # roadside_bin_red_2
                (10.0, 3.5),    # roadside_bin_red_3
            ],
            'blue': [
                (3.5, 12.0),    # roadside_bin_blue_1
                (-3.5, -12.0),  # roadside_bin_blue_2
                (-10.0, -3.5),  # roadside_bin_blue_3
            ],
        }
        
        # Subscribers
        self.waste_sub = self.create_subscription(
            String,
            '/detected_waste',
            self.waste_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/gcar/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for navigation targets
        self.nav_target_pub = self.create_publisher(Point, '/nav/target', 10)
        
        # Service clients (created when needed)
        self.arm_home_client = None
        self.arm_pick_client = None
        self.arm_place_internal_client = None
        self.arm_place_bin_client = None
        self.gazebo_pickup_client = None
        self.gazebo_place_client = None
        
        # State machine timer (1 Hz)
        self.sm_timer = self.create_timer(1.0, self.state_machine_step)
        
        self.get_logger().info('Pickup Coordinator Started')
        self.get_logger().info(f'State: {self.state}')
    
    def odom_callback(self, msg):
        """Update robot position and orientation."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def waste_callback(self, msg):
        """Handle waste detection."""
        # CRITICAL: Only start new workflow if currently IDLE
        if self.state != self.STATE_IDLE:
            # Already processing a workflow, ignore new detections
            self.get_logger().debug(f'Ignoring detection - already in state: {self.state}')
            return
        
        self.detected_waste_type = msg.data  # 'red_waste' or 'blue_waste'
        color = self.detected_waste_type.replace('_waste', '')
        
        # Use robot's CURRENT position + small forward offset (waste is detected in front of robot)
        # Camera is front-facing, so waste is approximately 0.5-1.0m in front
        forward_offset = 0.8  # meters in front of robot
        estimated_x = self.robot_x + forward_offset * math.cos(self.robot_yaw)
        estimated_y = self.robot_y + forward_offset * math.sin(self.robot_yaw)
        self.waste_position = (estimated_x, estimated_y)

        # --------------------------------------------------------------
        # Proximity filter (Ghost Fix near the bin)
        # --------------------------------------------------------------
        # If the estimated waste position is very close to the robot
        # AND also very close to the matching bin, we assume this is
        # just the cube we *just dropped* at the bin and ignore it.
        # Find distance from estimated waste position to the NEAREST bin of
        # this color (used only for \"ghost at bin\" filtering).
        candidate_bins = self.bin_locations[color]
        # Distance from robot to waste (in front of camera)
        dist_robot_to_waste = math.sqrt(
            (estimated_x - self.robot_x) ** 2 + (estimated_y - self.robot_y) ** 2
        )
        # Minimum distance from waste estimate to any bin of this color
        dist_waste_to_bin = min(
            math.sqrt((estimated_x - bx) ** 2 + (estimated_y - by) ** 2)
            for (bx, by) in candidate_bins
        )

        if dist_robot_to_waste < 0.5 and dist_waste_to_bin < 1.0:
            self.get_logger().info(
                'Detection appears to be at the bin (likely just dropped waste). '
                f'Ignoring detection: robot→waste={dist_robot_to_waste:.2f}m, '
                f'waste→bin={dist_waste_to_bin:.2f}m.'
            )
            # Reset detection and stay in IDLE
            self.detected_waste_type = None
            self.waste_position = None
            return
        
        self.get_logger().info(f'Detected {self.detected_waste_type}!')
        self.get_logger().info(f'Robot at: ({self.robot_x:.2f}, {self.robot_y:.2f}), yaw: {self.robot_yaw:.2f}')
        self.get_logger().info(f'Estimated waste position: ({self.waste_position[0]:.2f}, {self.waste_position[1]:.2f})')
        self.get_logger().info('Starting pickup workflow...')
        self.state = self.STATE_APPROACH_WASTE
        self.pickup_success = False  # Reset pickup status
    
    def state_machine_step(self):
        """Main state machine loop."""
        if self.state == self.STATE_IDLE:
            # Check if robot is carrying waste and manually approaching a bin
            current_time = time.time()
            if (self.carrying_waste_type is not None and 
                (current_time - self.last_bin_check_time) >= self.bin_check_interval):
                self.last_bin_check_time = current_time
                
                # Get candidate bins for the color we're carrying
                color = self.carrying_waste_type
                candidate_bins = self.bin_locations.get(color, [])
                
                # Find nearest bin and check if we're close enough
                nearest_bin = None
                nearest_dist = None
                for bx, by in candidate_bins:
                    dx = bx - self.robot_x
                    dy = by - self.robot_y
                    dist = math.sqrt(dx * dx + dy * dy)
                    if nearest_dist is None or dist < nearest_dist:
                        nearest_dist = dist
                        nearest_bin = (bx, by)
                
                # If within 1.5m of matching bin, automatically trigger PLACE
                if nearest_bin is not None and nearest_dist < 1.5:
                    self.get_logger().info(
                        f'Automatic drop triggered: Carrying {color} waste, '
                        f'near {color} bin at ({nearest_bin[0]:.2f}, {nearest_bin[1]:.2f}), '
                        f'distance: {nearest_dist:.2f}m'
                    )
                    # Set detected_waste_type temporarily so PLACE logic knows which color
                    self.detected_waste_type = f'{color}_waste'
                    self.state = self.STATE_PLACE
        
        elif self.state == self.STATE_APPROACH_WASTE:
            # Simple approach: just wait a bit for robot to get closer
            # In full implementation, could navigate to waste position
            if not hasattr(self, '_approach_start_time'):
                self.get_logger().info('State: APPROACH_WASTE')
                self._approach_start_time = time.time()
            
            if time.time() - self._approach_start_time >= 2.0:
                delattr(self, '_approach_start_time')
                self.state = self.STATE_ALIGN_TO_WASTE
        
        elif self.state == self.STATE_ALIGN_TO_WASTE:
            # Rotate robot to face waste before picking
            if self.waste_position is None:
                # Skip alignment if no waste position
                self.get_logger().warn('No waste position! Skipping alignment.')
                self.state = self.STATE_PICKUP
                return
            
            if not hasattr(self, '_align_start_time'):
                self.get_logger().info('State: ALIGN_TO_WASTE')
                self._align_start_time = time.time()
            
            dx = self.waste_position[0] - self.robot_x
            dy = self.waste_position[1] - self.robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            target_angle = math.atan2(dy, dx)
            
            # Calculate angle difference
            angle_diff = target_angle - self.robot_yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # Log alignment progress every 2 seconds
            if time.time() - self._align_start_time >= 2.0:
                self.get_logger().info(f'Aligning... angle_diff: {math.degrees(abs(angle_diff)):.1f}°, distance: {distance:.2f}m')
                self._align_start_time = time.time()
            
            # If aligned (within 0.2 rad ~ 11.5 degrees) OR very close, proceed to pickup
            if abs(angle_diff) < 0.2 or distance < 0.5:
                self.get_logger().info(f'Aligned to waste! Angle diff: {math.degrees(abs(angle_diff)):.1f}°, Distance: {distance:.2f}m')
                self.stop_robot()
                if hasattr(self, '_align_start_time'):
                    delattr(self, '_align_start_time')
                self.state = self.STATE_PICKUP
            else:
                # Rotate towards waste
                twist = Twist()
                angular_speed = 0.4 if abs(angle_diff) > 0.3 else 0.2  # Slower when close
                twist.angular.z = angular_speed if angle_diff > 0 else -angular_speed
                self.cmd_vel_pub.publish(twist)
        
        elif self.state == self.STATE_PICKUP:
            if not hasattr(self, '_pickup_start_time'):
                self.get_logger().info('State: PICKUP (arm down + delete model)')
                self._pickup_start_time = time.time()
                self._arm_pick_called = False
                self._gazebo_pickup_called = False
                self.pickup_success = False
                self._gazebo_pickup_future = None
                self._gazebo_pickup_request_time = None
                self._pickup_decided = False
            
            elapsed = time.time() - self._pickup_start_time
            
            # Call arm service after 0.5s
            if elapsed >= 0.5 and not self._arm_pick_called:
                self._call_arm_pick()
                self._arm_pick_called = True
            
            # Call Gazebo pickup service after 3.5s (arm should be down)
            if elapsed >= 3.5 and not self._gazebo_pickup_called:
                # IMPORTANT: do NOT block inside a timer callback with
                # spin_until_future_complete. We start an async call and
                # check completion on subsequent timer ticks.
                self._gazebo_pickup_future = self._call_gazebo_pickup_async()
                self._gazebo_pickup_request_time = time.time()
                self._gazebo_pickup_called = True
            
            # If we started the Gazebo pickup call, check if it finished
            if self._gazebo_pickup_called and self._gazebo_pickup_future is not None:
                if self._gazebo_pickup_future.done():
                    try:
                        resp = self._gazebo_pickup_future.result()
                        self.pickup_success = bool(resp and resp.success)
                        if self.pickup_success:
                            self.get_logger().info('Gazebo pickup succeeded!')
                        else:
                            self.get_logger().error(f'Gazebo pickup failed: {resp.message if resp else "No response"}')
                    except Exception as e:  # noqa: BLE001
                        self.pickup_success = False
                        self.get_logger().error(f'Gazebo pickup exception: {e}')
                    self._pickup_decided = True
                else:
                    # Wait up to 6 seconds for the service response.
                    # If it times out, treat as failure (gazebo_manager now
                    # does its own environment-based verification).
                    if (
                        self._gazebo_pickup_request_time is not None and
                        (time.time() - self._gazebo_pickup_request_time) > 6.0
                    ):
                        self.pickup_success = False
                        self.get_logger().error('Gazebo pickup service call timeout!')
                        self._pickup_decided = True
            
            # DECISION:
            # Previously we decided at a fixed time (4.5s). That caused a bug:
            # gazebo_manager can take longer to respond, so we aborted before
            # the service reply arrived. Now we only decide after either:
            # - the pickup service future completes, or
            # - the pickup service times out.
            #
            # Still keep a hard upper bound so we don't hang forever.
            if not self._pickup_decided and elapsed >= 12.0:
                self.pickup_success = False
                self._pickup_decided = True
                self.get_logger().error('Pickup overall timeout (no service response). Aborting.')
            
            if self._pickup_decided:
                if self.pickup_success:
                    # Extract color from detected_waste_type and store as carrying state
                    color = self.detected_waste_type.replace('_waste', '') if self.detected_waste_type else None
                    self.carrying_waste_type = color
                    self.get_logger().info(f'Carrying {color} waste (state tracked)')
                    
                    # Raise arm to a mid/high "carry" pose (PLACE_INTERNAL)
                    # so it looks like the robot is holding the waste up and
                    # navigation is easier than with the arm near the ground.
                    self.get_logger().info(
                        'Pickup successful! Raising arm to CARRY (PLACE_INTERNAL) pose before navigating to bin.'
                    )
                    self._call_arm_place_internal()
                    time.sleep(2.0)  # brief pause to let the arm move visually
                    self.get_logger().info('Arm in carry pose. Proceeding to bin.')
                    delattr(self, '_pickup_start_time')
                    self.state = self.STATE_NAVIGATE_TO_BIN
                else:
                    self.get_logger().error('Pickup verification failed. Aborting.')
                    delattr(self, '_pickup_start_time')
                    self.state = self.STATE_IDLE
                    self.detected_waste_type = None
                    self.waste_position = None
        
        elif self.state == self.STATE_NAVIGATE_TO_BIN:
            # Initialize navigation on first entry
            if self.navigation_target is None:
                color = self.detected_waste_type.replace('_waste', '')  # 'red' or 'blue'
                # Choose the NEAREST bin of the appropriate color from our
                # catalog, based on the robot's current odom pose.
                candidate_bins = self.bin_locations[color]
                nearest_bin = None
                nearest_dist = None
                for bx, by in candidate_bins:
                    dx = bx - self.robot_x
                    dy = by - self.robot_y
                    dist = math.sqrt(dx * dx + dy * dy)
                    if nearest_dist is None or dist < nearest_dist:
                        nearest_dist = dist
                        nearest_bin = (bx, by)

                self.navigation_target = nearest_bin
                # Track best (minimum) distance seen so far to detect if we
                # start moving away from the bin.
                self._nav_min_distance = nearest_dist
                self.last_nav_log_time = time.time()
                
                self.get_logger().info('State: NAVIGATE_TO_BIN')
                self.get_logger().info(
                    f'Driving to NEAREST {color} bin at '
                    f'({nearest_bin[0]:.2f}, {nearest_bin[1]:.2f}), '
                    f'starting distance {nearest_dist:.2f}m'
                )
                
                # Publish navigation target
                target_msg = Point()
                target_msg.x = nearest_bin[0]
                target_msg.y = nearest_bin[1]
                target_msg.z = 0.0
                self.nav_target_pub.publish(target_msg)
            
            # Check if robot has reached bin (non-blocking)
            distance_to_bin = math.sqrt(
                (self.robot_x - self.navigation_target[0])**2 + 
                (self.robot_y - self.navigation_target[1])**2
            )

            # Update minimum distance seen so far
            if hasattr(self, '_nav_min_distance'):
                if distance_to_bin < self._nav_min_distance:
                    self._nav_min_distance = distance_to_bin
            
            # Log progress every 2 seconds
            current_time = time.time()
            if current_time - self.last_nav_log_time >= 2.0:
                self.get_logger().info(f'Driving to bin... Distance remaining: {distance_to_bin:.2f}m')
                self.last_nav_log_time = current_time
            
            # Check if arrived
            if distance_to_bin < 1.0:  # Within 1 meter of bin
                color = self.detected_waste_type.replace('_waste', '')
                self.get_logger().info(f'Arrived at {color} bin! Distance: {distance_to_bin:.2f}m')
                self.navigation_target = None  # Reset for next navigation
                if hasattr(self, '_nav_min_distance'):
                    delattr(self, '_nav_min_distance')
                self.state = self.STATE_PLACE
                return

            # SAFETY: if we have moved significantly farther away from the bin
            # than the closest we have ever been (e.g. > 5m worse), abort
            # navigation to avoid driving out of the city.
            if hasattr(self, '_nav_min_distance'):
                if distance_to_bin > (self._nav_min_distance + 5.0):
                    self.get_logger().warn(
                        'Navigation distance to bin has increased by more than 5m '
                        'from the closest point reached. Aborting NAVIGATE_TO_BIN '
                        'to avoid leaving the city. Handing control back to operator.'
                    )
                    self.stop_robot()
                    self.navigation_target = None
                    delattr(self, '_nav_min_distance')
                    self.state = self.STATE_IDLE
                    self.detected_waste_type = None
                    self.waste_position = None
                    # NOTE: Preserve carrying_waste_type so manual bin approach can trigger drop
                    self.get_logger().info(
                        f'Navigation aborted. Still carrying {self.carrying_waste_type} waste. '
                        'Manual drive to bin will trigger automatic drop.'
                    )
        
        elif self.state == self.STATE_PLACE:
            # Check if this was triggered by automatic proximity check or normal navigation
            if self.detected_waste_type and self.carrying_waste_type:
                self.get_logger().info(
                    f'State: PLACE (arm to bin + spawn model) - '
                    f'{"Automatic drop triggered by bin proximity" if self.navigation_target is None else "Normal navigation flow"}'
                )
            else:
                self.get_logger().info('State: PLACE (arm to bin + spawn model)')
            # Call arm service to reach bin
            self._call_arm_place_bin()
            time.sleep(3.0)
            # Call Gazebo service to spawn waste at bin
            self._call_gazebo_place()
            time.sleep(1.0)
            # Immediately hand control back to the operator:
            # 1) publish an explicit stop to /gcar/cmd_vel
            # 2) go back to IDLE so teleop is the only active controller.
            self.stop_robot()  # flush any residual autonomous velocity
            self.get_logger().info('[pickup_coordinator]: Nav2/simple navigator stopped. Teleop control is now ACTIVE.')
            self.state = self.STATE_IDLE
            self.detected_waste_type = None
            self.waste_position = None
            self.pickup_success = False
            # Clear carrying state after successful placement
            self.carrying_waste_type = None
            self.get_logger().info('Placement complete. Carrying state cleared.')
    
    def _call_arm_home(self):
        """Call arm home service."""
        if self.arm_home_client is None:
            self.arm_home_client = self.create_client(Trigger, '/arm/go_home')
        
        if self.arm_home_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.arm_home_client.call_async(req)
            self.get_logger().info('Called /arm/go_home')
    
    def _call_arm_pick(self):
        """Call arm pick service."""
        if self.arm_pick_client is None:
            self.arm_pick_client = self.create_client(Trigger, '/arm/go_pick')
        
        if self.arm_pick_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.arm_pick_client.call_async(req)
            self.get_logger().info('Called /arm/go_pick')
    
    def _call_arm_place_internal(self):
        """Call arm PLACE_INTERNAL service (used as mid/high carry pose)."""
        if self.arm_place_internal_client is None:
            self.arm_place_internal_client = self.create_client(Trigger, '/arm/go_place')
        
        if self.arm_place_internal_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.arm_place_internal_client.call_async(req)
            self.get_logger().info('Called /arm/go_place (carry pose)')
    
    def _call_arm_place_bin(self):
        """Call arm place bin service."""
        if self.arm_place_bin_client is None:
            self.arm_place_bin_client = self.create_client(Trigger, '/arm/go_place_bin')
        
        if self.arm_place_bin_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.arm_place_bin_client.call_async(req)
            self.get_logger().info('Called /arm/go_place_bin')
    
    def _call_gazebo_pickup(self):
        """Call Gazebo pickup service (async)."""
        if self.gazebo_pickup_client is None:
            self.gazebo_pickup_client = self.create_client(Trigger, '/gazebo/pickup_waste')
        
        if self.gazebo_pickup_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.gazebo_pickup_client.call_async(req)
            self.get_logger().info('Called /gazebo/pickup_waste')
    
    def _call_gazebo_pickup_async(self):
        """Start Gazebo pickup service call and return future (non-blocking)."""
        if self.gazebo_pickup_client is None:
            self.gazebo_pickup_client = self.create_client(Trigger, '/gazebo/pickup_waste')
        
        if not self.gazebo_pickup_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Gazebo pickup service not available!')
            return None
        
        req = Trigger.Request()
        future = self.gazebo_pickup_client.call_async(req)
        self.get_logger().info('Called /gazebo/pickup_waste')
        return future
    
    def stop_robot(self):
        """Stop robot movement."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def _call_gazebo_place(self):
        """Call Gazebo place service."""
        if self.gazebo_place_client is None:
            self.gazebo_place_client = self.create_client(Trigger, '/gazebo/place_waste')
        
        if self.gazebo_place_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.gazebo_place_client.call_async(req)
            self.get_logger().info('Called /gazebo/place_waste')


def main(args=None):
    rclpy.init(args=args)
    node = PickupCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

