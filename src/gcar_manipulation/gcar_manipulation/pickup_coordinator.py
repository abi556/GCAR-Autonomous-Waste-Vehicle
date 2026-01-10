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
import math
import time


class PickupCoordinator(Node):
    """Simple state machine coordinator for waste pickup workflow."""
    
    # State definitions
    STATE_IDLE = 'IDLE'
    STATE_APPROACH_WASTE = 'APPROACH_WASTE'
    STATE_PICKUP = 'PICKUP'
    STATE_NAVIGATE_TO_BIN = 'NAVIGATE_TO_BIN'
    STATE_PLACE = 'PLACE'
    STATE_RETURN_HOME = 'RETURN_HOME'
    
    def __init__(self):
        super().__init__('pickup_coordinator')
        
        # Current state
        self.state = self.STATE_IDLE
        self.detected_waste_type = None  # 'red_waste' or 'blue_waste'
        
        # Robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Waste and bin locations (hardcoded for simplicity)
        self.waste_locations = {
            'red': (1.5, 1.0),
            'blue': (-1.5, 1.0),
        }
        self.bin_locations = {
            'red': (3.5, 8.0),
            'blue': (3.5, 12.0),
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
        
        # Service clients (created when needed)
        self.arm_home_client = None
        self.arm_pick_client = None
        self.arm_place_bin_client = None
        self.gazebo_pickup_client = None
        self.gazebo_place_client = None
        
        # State machine timer (1 Hz)
        self.sm_timer = self.create_timer(1.0, self.state_machine_step)
        
        self.get_logger().info('Pickup Coordinator Started')
        self.get_logger().info(f'State: {self.state}')
    
    def odom_callback(self, msg):
        """Update robot position."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
    
    def waste_callback(self, msg):
        """Handle waste detection."""
        if self.state == self.STATE_IDLE:
            self.detected_waste_type = msg.data  # 'red_waste' or 'blue_waste'
            color = self.detected_waste_type.replace('_waste', '')
            self.get_logger().info(f'Detected {self.detected_waste_type}! Starting pickup workflow...')
            self.state = self.STATE_APPROACH_WASTE
    
    def state_machine_step(self):
        """Main state machine loop."""
        if self.state == self.STATE_IDLE:
            # Waiting for waste detection
            pass
        
        elif self.state == self.STATE_APPROACH_WASTE:
            self.get_logger().info('State: APPROACH_WASTE')
            # In full implementation, would call simple_navigator service
            # For now, just wait and assume we're close
            time.sleep(2.0)
            self.state = self.STATE_PICKUP
        
        elif self.state == self.STATE_PICKUP:
            self.get_logger().info('State: PICKUP (arm down + delete model)')
            # Call arm service to lower
            self._call_arm_pick()
            time.sleep(3.0)
            # Call Gazebo service to delete waste
            self._call_gazebo_pickup()
            time.sleep(1.0)
            self.state = self.STATE_NAVIGATE_TO_BIN
        
        elif self.state == self.STATE_NAVIGATE_TO_BIN:
            self.get_logger().info('State: NAVIGATE_TO_BIN')
            # In full implementation, would navigate to bin
            time.sleep(2.0)
            self.state = self.STATE_PLACE
        
        elif self.state == self.STATE_PLACE:
            self.get_logger().info('State: PLACE (arm to bin + spawn model)')
            # Call arm service to reach bin
            self._call_arm_place_bin()
            time.sleep(3.0)
            # Call Gazebo service to spawn waste at bin
            self._call_gazebo_place()
            time.sleep(1.0)
            self.state = self.STATE_RETURN_HOME
        
        elif self.state == self.STATE_RETURN_HOME:
            self.get_logger().info('State: RETURN_HOME')
            self._call_arm_home()
            time.sleep(3.0)
            self.get_logger().info('Workflow complete! Returning to IDLE.')
            self.state = self.STATE_IDLE
            self.detected_waste_type = None
    
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
    
    def _call_arm_place_bin(self):
        """Call arm place bin service."""
        if self.arm_place_bin_client is None:
            self.arm_place_bin_client = self.create_client(Trigger, '/arm/go_place_bin')
        
        if self.arm_place_bin_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.arm_place_bin_client.call_async(req)
            self.get_logger().info('Called /arm/go_place_bin')
    
    def _call_gazebo_pickup(self):
        """Call Gazebo pickup service."""
        if self.gazebo_pickup_client is None:
            self.gazebo_pickup_client = self.create_client(Trigger, '/gazebo/pickup_waste')
        
        if self.gazebo_pickup_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.gazebo_pickup_client.call_async(req)
            self.get_logger().info('Called /gazebo/pickup_waste')
    
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

