#!/usr/bin/env python3
"""Simple navigation node for driving to target coordinates.

This node provides a service to drive the robot to a target (x, y) position
using simple proportional control with odometry feedback.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import math


class SimpleNavigator(Node):
    """Basic navigation node using proportional control."""
    
    def __init__(self):
        super().__init__('simple_navigator')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/gcar/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/gcar/odom',
            self.odom_callback,
            10
        )
        
        # Current robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Navigation parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.position_tolerance = 0.2  # meters
        self.angle_tolerance = 0.1  # radians
        
        # Target position (set by service call)
        self.target_x = None
        self.target_y = None
        self.navigating = False
        
        # Control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Subscribe to navigation target commands
        self.target_sub = self.create_subscription(
            Point,
            '/nav/target',
            self.target_callback,
            10
        )
        
        self.get_logger().info('Simple Navigator Node Started')
        self.get_logger().info('Listening for targets on: /nav/target')
    
    def odom_callback(self, msg):
        """Update robot position from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def target_callback(self, msg):
        """Receive navigation target from topic.
        
        Args:
            msg: geometry_msgs/Point with x, y coordinates (z ignored)
        """
        self.target_x = msg.x
        self.target_y = msg.y
        self.navigating = True
        self.get_logger().info(f'New navigation target: ({self.target_x:.2f}, {self.target_y:.2f})')
        self.get_logger().info(f'Current position: ({self.robot_x:.2f}, {self.robot_y:.2f})')
    
    def control_loop(self):
        """Main control loop for navigation."""
        if not self.navigating or self.target_x is None:
            return
        
        # Calculate distance and angle to target
        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        
        # Angle difference (wrapped to [-pi, pi])
        angle_diff = target_angle - self.robot_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Check if reached target
        if distance < self.position_tolerance:
            self.get_logger().info('Reached target!')
            self.stop_robot()
            self.navigating = False
            return
        
        # Proportional control
        twist = Twist()
        
        # If facing wrong direction, rotate first
        if abs(angle_diff) > self.angle_tolerance:
            twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            twist.linear.x = 0.0
        else:
            # Move forward
            twist.linear.x = min(self.linear_speed, distance)
            twist.angular.z = 0.5 * angle_diff  # Small correction
        
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

