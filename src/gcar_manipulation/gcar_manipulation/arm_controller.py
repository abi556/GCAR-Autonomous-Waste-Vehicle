#!/usr/bin/env python3
"""Arm controller node with preset pick and place poses."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger
import time


class ArmController(Node):
    """Control GCAR robot arm with preset poses.
    
    Provides services to move the arm to predefined positions:
    - HOME: Stowed position
    - PICK_SIDE: Position to pick waste from ground
    - PLACE_INTERNAL: Position to drop waste in collection box
    """
    
    # Preset joint positions [arm_base_joint, shoulder_joint, elbow_joint]
    # Angles in radians, tuned for robot geometry (arm base 0.25m above ground)
    # POSITIVE shoulder/elbow angles = reach forward and down
    POSES = {
        'home': [0.0, 0.0, 0.0],                    # Stowed upright position (all joints straight)
        'pick_front': [0.0, 1.35, 1.25],            # FIXED: POSITIVE angles to reach FORWARD where camera sees
        'place_internal': [0.0, 0.65, 0.85],        # Face forward, reach over chassis to drop waste
        'place_bin': [1.57, 0.3, 0.4],              # Rotate 90° right, reach out ~0.5m high to drop into bin
    }
    
    def __init__(self):
        super().__init__('arm_controller')
        
        # Action client for joint trajectory controller
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gcar/arm_controller/follow_joint_trajectory'
        )
        
        # Joint names (must match URDF and controller config)
        self.joint_names = [
            'arm_base_joint',
            'shoulder_joint',
            'elbow_joint'
        ]
        
        # Services for each preset pose
        self.srv_home = self.create_service(
            Trigger,
            '/arm/go_home',
            self.go_home_callback
        )
        
        self.srv_pick = self.create_service(
            Trigger,
            '/arm/go_pick',
            self.go_pick_callback
        )
        
        self.srv_place = self.create_service(
            Trigger,
            '/arm/go_place',
            self.go_place_callback
        )
        
        self.srv_place_bin = self.create_service(
            Trigger,
            '/arm/go_place_bin',
            self.go_place_bin_callback
        )
        
        self.get_logger().info('Arm Controller Node Started')
        self.get_logger().info('Available services:')
        self.get_logger().info('  - /arm/go_home      : Move to HOME pose')
        self.get_logger().info('  - /arm/go_pick      : Move to PICK_FRONT pose (where camera detects)')
        self.get_logger().info('  - /arm/go_place     : Move to PLACE_INTERNAL pose')
        self.get_logger().info('  - /arm/go_place_bin : Move to PLACE_BIN pose (drop into world bins)')
        
        # Wait for action server to be available
        self.get_logger().info('Waiting for arm_controller action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Connected to arm_controller!')
    
    def go_home_callback(self, request, response):
        """Service callback to move arm to HOME position."""
        self.get_logger().info('Moving arm to HOME position...')
        success = self.move_to_pose('home', duration=3.0)
        
        response.success = success
        response.message = 'Moved to HOME' if success else 'Failed to move to HOME'
        return response
    
    def go_pick_callback(self, request, response):
        """Service callback to move arm to PICK_FRONT position."""
        self.get_logger().info('Moving arm to PICK_FRONT position (where camera sees waste)...')
        success = self.move_to_pose('pick_front', duration=4.0)
        
        response.success = success
        response.message = 'Moved to PICK_FRONT' if success else 'Failed to move to PICK_FRONT'
        return response
    
    def go_place_callback(self, request, response):
        """Service callback to move arm to PLACE_INTERNAL position."""
        self.get_logger().info('Moving arm to PLACE_INTERNAL position...')
        success = self.move_to_pose('place_internal', duration=4.0)
        
        response.success = success
        response.message = 'Moved to PLACE_INTERNAL' if success else 'Failed to move to PLACE_INTERNAL'
        return response
    
    def go_place_bin_callback(self, request, response):
        """Service callback to move arm to PLACE_BIN position."""
        self.get_logger().info('Moving arm to PLACE_BIN position (drop into world bin)...')
        success = self.move_to_pose('place_bin', duration=4.0)
        
        response.success = success
        response.message = 'Moved to PLACE_BIN' if success else 'Failed to move to PLACE_BIN'
        return response
    
    def move_to_pose(self, pose_name, duration=3.0):
        """Move arm to a preset pose using fire-and-forget approach.
        
        Args:
            pose_name: Name of the preset pose ('home', 'pick_side', 'place_internal')
            duration: Time in seconds to complete the motion
            
        Returns:
            bool: True if goal was sent successfully, False otherwise
        """
        if pose_name not in self.POSES:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            return False
        
        target_positions = self.POSES[pose_name]
        
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        
        # Build trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Single waypoint (target position)
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        # Send goal without waiting (fire-and-forget)
        # The action server will execute it, and coordinator will sleep to wait
        self.get_logger().info(f'Sending trajectory to {pose_name}: {target_positions}')
        self.action_client.send_goal_async(goal_msg)
        
        # Return True immediately - the coordinator will handle timing
        return True


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = ArmController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
