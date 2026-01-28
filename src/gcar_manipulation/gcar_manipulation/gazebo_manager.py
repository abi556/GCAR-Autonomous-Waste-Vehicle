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
from nav_msgs.msg import Odometry
import math


class GazeboManager(Node):
    """Manager for spawning and deleting waste models in Gazebo."""
    
    def __init__(self):
        super().__init__('gazebo_manager')
        
        # Carrying state tracking
        self.carrying_waste = False
        self.carried_waste_type = None  # 'red' or 'blue'
        self.last_deleted_name = None   # exact Gazebo model name last removed

        # Waste tracking:
        # - active_waste_names: ground waste cubes that can still be picked
        # - picked_waste_names: cubes that have been picked (and possibly
        #   re-spawned at the bin with a *_recycled suffix)
        self.active_waste_names = set([
            'waste_red_1', 'waste_red_2', 'waste_red_3', 'waste_red_4',
            'waste_blue_1', 'waste_blue_2', 'waste_blue_3', 'waste_blue_4',
        ])
        self.picked_waste_names = set()
        
        # Service clients for Gazebo
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Track latest model list from Gazebo so we can optionally verify
        # deletions. Note: on some setups /gazebo/model_states may be quiet,
        # so we do NOT rely on it for proximity any more.
        self.latest_model_names = None
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10,
        )

        # Robot odometry for proximity selection (always available in this setup)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_received = False
        self.odom_sub = self.create_subscription(
            Odometry,
            '/gcar/odom',
            self.odom_callback,
            10,
        )

        # Static catalog of ground waste locations (taken from city.world)
        # Used for proximity-based selection when picking up nearest waste.
        self.waste_catalog = {
            'waste_red_1':  (1.5,  1.0),
            'waste_red_2':  (1.0, -3.0),
            'waste_red_3':  (5.0,  1.0),
            'waste_red_4':  (3.0, -4.0),
            'waste_blue_1': (-1.5, 1.0),
            'waste_blue_2': (-1.0, 4.0),
            'waste_blue_3': (-5.0,-1.0),
            'waste_blue_4': (-3.0,-4.0),
        }
        
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
        """Store the latest list of model names from Gazebo (optional)."""
        self.latest_model_names = set(msg.name)

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry for proximity selection."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.odom_received = True
    
    def pickup_waste_callback(self, request, response):
        """Simulate picking up waste by deleting the NEAREST ground waste model.

        Behavior:
        1. Use /gcar/odom to know the robot pose.
        2. Among all entries in waste_catalog that are still in active_waste_names
           (i.e. not previously picked), find the closest one within a 1.0 m radius.
        3. Delete that specific model using DeleteEntity.
        4. Mark it as picked so it is never targeted again.

        NOTE: We previously attempted to use /gazebo/model_states for proximity,
        but on this setup that topic is not reliably publishing. Using odometry +
        static waste coordinates from city.world is robust enough for the demo.
        """
        if self.carrying_waste:
            response.success = False
            response.message = 'Already carrying waste! Drop it first.'
            return response

        if not self.odom_received:
            response.success = False
            response.message = 'No odometry received yet; cannot select nearest waste.'
            self.get_logger().warn(response.message)
            return response

        rx = self.robot_x
        ry = self.robot_y

        # Scan catalog for nearest eligible waste within 1.0 m
        nearest_name = None
        nearest_dist = None
        pickup_radius = 1.0  # meters
        for name, (wx, wy) in self.waste_catalog.items():
            if name not in self.active_waste_names:
                continue
            dx = wx - rx
            dy = wy - ry
            dist = math.sqrt(dx * dx + dy * dy)
            if dist <= pickup_radius and (nearest_dist is None or dist < nearest_dist):
                nearest_name = name
                nearest_dist = dist

        if nearest_name is None:
            response.success = False
            response.message = 'No waste entity within pickup radius.'
            self.get_logger().info(response.message)
            return response

        model_name = nearest_name

        # Global tracker guard: only allow pickup if this waste is known to
        # be active in the world. This avoids \"ghost pickups\" of already
        # collected or non-existent models.
        if model_name not in self.active_waste_names:
            response.success = False
            response.message = f'{model_name} is not active_in_world; refusing pickup.'
            self.get_logger().warn(response.message)
            return response
        
        # Delete the waste model and *verify* that it actually disappeared
        # using Gazebo's model list.
        delete_req = DeleteEntity.Request()
        delete_req.name = model_name
        future = self.delete_client.call_async(delete_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        # Log raw delete result from Gazebo (for debugging), but don't rely
        # on it as the only truth source.
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Gazebo delete reported success for {model_name}')
        else:
            self.get_logger().warn(
                f'Gazebo delete service reported failure for {model_name} '
                f'({future.result().status_message if future.result() else "no result"})'
            )
        
        # Regardless of the immediate delete result, confirm via
        # /gazebo/model_states that the model name actually disappears.
        model_gone = False
        for _ in range(15):  # up to ~1.5s total
            # Allow this node to process a /gazebo/model_states update
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_model_names is not None:
                if model_name not in self.latest_model_names:
                    model_gone = True
                    break
        
        if not model_gone:
            # For this magic demo, Gazebo's DeleteEntity sometimes doesn't
            # give us a reliable response, and /gazebo/model_states can lag.
            # However, from the user's point of view the cube has visibly
            # disappeared and pickup should proceed. So we WARN but treat
            # this as success instead of blocking the whole workflow.
            self.get_logger().warn(
                f'Could not confirm removal of {model_name} in Gazebo model list; '
                'assuming pickup succeeded for magic demo.'
            )

        # Update global tracker: this specific waste has now been removed
        # from the ground and is considered "picked".
        if model_name in self.active_waste_names:
            self.active_waste_names.remove(model_name)
            self.get_logger().info(
                f'Removed {model_name} from active_in_world. '
                f'Active ground wastes: {len(self.active_waste_names)}'
            )
        self.picked_waste_names.add(model_name)
        
        # Treat as successful pickup and remember which color/name we took
        self.carrying_waste = True
        if 'red' in model_name:
            self.carried_waste_type = 'red'
        elif 'blue' in model_name:
            self.carried_waste_type = 'blue'
        else:
            self.carried_waste_type = None
        self.last_deleted_name = model_name
        self.get_logger().info(f'Picked up {model_name} (world tracker updated)')
        
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

        # Derive a stable recycled name from the last deleted waste
        # e.g. waste_red_4 -> waste_red_4_recycled
        if self.last_deleted_name:
            model_name = f'{self.last_deleted_name}_recycled'
        else:
            # Fallback if somehow we lost history
            model_name = 'waste_recycled'

        # Do NOT allow spawning if this recycled name is already present.
        if model_name in self.picked_waste_names:
            self.get_logger().warn(
                f'Refusing to spawn {model_name}: already in picked_waste_names.'
            )
            response.success = False
            response.message = f'{model_name} already exists as recycled; spawn blocked.'
            self.carrying_waste = False
            self.carried_waste_type = None
            return response
        
        # Create spawn request
        spawn_req = SpawnEntity.Request()
        spawn_req.name = model_name
        spawn_req.xml = self._generate_waste_sdf(self.carried_waste_type)
        spawn_req.initial_pose = Pose()
        spawn_req.initial_pose.position.x = bin_x
        spawn_req.initial_pose.position.y = bin_y
        spawn_req.initial_pose.position.z = 0.55  # Slightly above (now shorter) bin rim
        
        future = self.spawn_client.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and future.result().success:
            # Normal happy-path: Gazebo confirms the spawn
            self.get_logger().info(f'Dropped {model_name} at ({bin_x}, {bin_y})')
            response.success = True
            response.message = f'Dropped waste at bin ({bin_x}, {bin_y})'
            # Track this recycled model so we never try to spawn it again.
            self.picked_waste_names.add(model_name)
            self.get_logger().info(
                f'Registered {model_name} as recycled. Picked set size: {len(self.picked_waste_names)}'
            )
        else:
            # For the magic demo, do NOT block the workflow if Gazebo
            # reports a failure or times out. From the operator's point
            # of view, the important part is that the robot *acted* like
            # it dropped the waste at the bin, not whether the cube
            # visually appears every time.
            self.get_logger().warn(
                f'Failed to spawn {model_name} (or no response). '
                'Keeping it out of picked_waste_names to avoid ghost duplicates.'
            )
            response.success = False
            response.message = 'Failed to spawn recycled waste (tracker unchanged)'
        
        # In all cases, clear the carrying flag so that subsequent
        # /gazebo/pickup_waste calls are allowed.
        self.carrying_waste = False
        self.carried_waste_type = None
        
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

