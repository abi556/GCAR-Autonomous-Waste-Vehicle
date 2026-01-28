#!/usr/bin/env python3
"""Gazebo model manager for magic pickup/place simulation.

This node provides services to:
- Delete waste models from Gazebo (simulates picking up)
- Spawn waste models in Gazebo (simulates dropping)
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import math


class GazeboManager(Node):
    """Manager for spawning and deleting waste models in Gazebo."""
    
    def __init__(self):
        super().__init__('gazebo_manager')
        
        # State tracking
        self.carrying_waste = False
        self.carried_waste_type = None  # 'red' or 'blue'
        self.waste_counter = 0  # For generating unique model names
        
        # Service clients for Gazebo
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Track latest model list from Gazebo to verify deletions
        self.latest_model_names = None
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10,
        )
        
        # Wait for Gazebo services
        self.get_logger().info('Waiting for Gazebo services...')
        self.delete_client.wait_for_service(timeout_sec=5.0)
        self.spawn_client.wait_for_service(timeout_sec=5.0)
        
        # Services for pickup/place actions
        self.srv_pickup = self.create_service(
            Trigger,
            '/gazebo/pickup_waste',
            self.pickup_waste_callback
        )
        
        self.srv_place = self.create_service(
            Trigger,
            '/gazebo/place_waste',
            self.place_waste_callback
        )
        
        self.get_logger().info('Gazebo Manager Node Started')
        self.get_logger().info('Services:')
        self.get_logger().info('  - /gazebo/pickup_waste : Delete waste model (magic pickup)')
        self.get_logger().info('  - /gazebo/place_waste  : Spawn waste model (magic place)')
        self.get_logger().info(f'Carrying waste: {self.carrying_waste}')
    
    def model_states_callback(self, msg: ModelStates):
        """Store the latest list of model names from Gazebo."""
        self.latest_model_names = set(msg.name)
    
    def pickup_waste_callback(self, request, response):
        """Simulate picking up waste by deleting nearest waste model.
        
        In a real implementation, this would:
        1. Find the nearest waste model to the robot
        2. Delete it from Gazebo
        3. Set carrying_waste flag
        """
        if self.carrying_waste:
            response.success = False
            response.message = 'Already carrying waste! Drop it first.'
            return response
        
        # For simplicity, we'll assume the closest waste is "waste_red_1"
        # In a more advanced version, this would use robot position to find
        # and delete the *actual* nearest waste model.
        model_name = 'waste_red_1'  # TODO: Find nearest waste dynamically
        
        # Delete the waste model and *verify* that it actually disappeared
        # using Gazebo's model list.
        delete_req = DeleteEntity.Request()
        delete_req.name = model_name
        future = self.delete_client.call_async(delete_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.result() is None or not future.result().success:
            self.get_logger().warn(f'Gazebo delete service reported failure for {model_name}')
            response.success = False
            response.message = f'Failed to delete {model_name}'
            return response
        
        # At this point Gazebo says the delete succeeded. To be robust, also
        # confirm via /gazebo/model_states that the model name disappears.
        model_gone = False
        for _ in range(15):  # up to ~1.5s total
            # Allow this node to process a /gazebo/model_states update
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_model_names is not None:
                if model_name not in self.latest_model_names:
                    model_gone = True
                    break
        
        if not model_gone:
            self.get_logger().warn(
                f'DeleteEntity succeeded but {model_name} still appears in model list.'
            )
            response.success = False
            response.message = f'{model_name} still present after delete'
            return response
        
        # Verified gone from world → treat as successful pickup
        self.carrying_waste = True
        self.carried_waste_type = 'red'  # In a dynamic version, derive from model_name
        self.get_logger().info(f'Picked up {model_name} (verified removed from Gazebo)')
        
        response.success = True
        response.message = f'Picked up {model_name}'
        return response
    
    def place_waste_callback(self, request, response):
        """Simulate placing waste by spawning a new model at bin location.
        
        In a real implementation, this would:
        1. Use robot position to find nearest matching bin
        2. Spawn waste model at bin location
        3. Clear carrying_waste flag
        """
        if not self.carrying_waste:
            response.success = False
            response.message = 'Not carrying any waste!'
            return response
        
        # For simplicity, spawn at a fixed location (red bin at 3.5, 8.0)
        # In full implementation, this would use robot position and carried_waste_type
        bin_x = 3.5
        bin_y = 8.0
        
        # Generate unique model name
        self.waste_counter += 1
        model_name = f'dropped_waste_{self.waste_counter}'
        
        # Create spawn request
        spawn_req = SpawnEntity.Request()
        spawn_req.name = model_name
        spawn_req.xml = self._generate_waste_sdf(self.carried_waste_type)
        spawn_req.initial_pose = Pose()
        spawn_req.initial_pose.position.x = bin_x
        spawn_req.initial_pose.position.y = bin_y
        spawn_req.initial_pose.position.z = 0.7  # Above bin rim
        
        future = self.spawn_client.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Dropped {model_name} at ({bin_x}, {bin_y})')
            self.carrying_waste = False
            self.carried_waste_type = None
            response.success = True
            response.message = f'Dropped waste at bin ({bin_x}, {bin_y})'
        else:
            self.get_logger().warn(f'Failed to spawn {model_name}')
            response.success = False
            response.message = 'Failed to spawn waste'
        
        return response
    
    def _generate_waste_sdf(self, color):
        """Generate SDF XML for a waste cube of given color."""
        if color == 'red':
            ambient = "0.9 0.1 0.1 1"
            diffuse = "1.0 0.2 0.2 1"
        elif color == 'blue':
            ambient = "0.05 0.15 0.9 1"
            diffuse = "0.10 0.25 1.0 1"
        else:
            ambient = "0.5 0.5 0.5 1"
            diffuse = "0.6 0.6 0.6 1"
        
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


def main(args=None):
    rclpy.init(args=args)
    node = GazeboManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

