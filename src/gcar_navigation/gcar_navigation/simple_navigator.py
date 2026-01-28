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
from std_msgs.msg import Bool
import math
import time


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
        
        # Navigation parameters - tuned for faster autonomous runs
        self.linear_speed = 2.2  # m/s (faster forward motion)
        self.angular_speed = 1.3  # rad/s (snappy but controllable turns)
        self.position_tolerance = 0.3  # meters (slightly increased for easier arrival)
        self.angle_tolerance = 0.15  # radians (slightly increased)
        
        # Target position (set by service call)
        self.target_x = None
        self.target_y = None
        self.navigating = False
        # Teleop override state
        self.teleop_active = False
        self.last_teleop_time = None
        
        # Control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Subscribe to navigation target commands
        self.target_sub = self.create_subscription(
            Point,
            '/nav/target',
            self.target_callback,
            10
        )
        # Subscribe to teleop activity flag so we can yield control when
        # the operator starts driving manually.
        self.teleop_sub = self.create_subscription(
            Bool,
            '/control/teleop_active',
            self.teleop_active_callback,
            10,
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
    
    def teleop_active_callback(self, msg: Bool):
        """Receive teleop activity flag from WASD node.
        
        When teleop is active, we should immediately stop navigating and
        yield control of /gcar/cmd_vel to the operator.
        """
        self.teleop_active = bool(msg.data)
        if self.teleop_active:
            self.last_teleop_time = time.time()
    
    def control_loop(self):
        """Main control loop for navigation."""
        # If teleop is active (or was very recently), completely yield:
        # stop the robot, clear navigating, and do not publish any commands.
        now = time.time()
        if self.teleop_active or (
            self.last_teleop_time is not None and (now - self.last_teleop_time) < 0.5
        ):
            if self.navigating:
                self.get_logger().debug('Teleop active - stopping navigator and yielding control.')
            self.stop_robot()
            self.navigating = False
            return

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
        
        # Safety buffer: stop when "close enough" to avoid touching the target (e.g., bin)
        safe_distance = 0.8  # meters - do not drive closer than this
        if distance < safe_distance:
            self.get_logger().info(
                f'Reached safe distance to target ({distance:.2f} m). Stopping to avoid collision.'
            )
            self.stop_robot()
            self.navigating = False
            return
        
        # Proportional control
        twist = Twist()
        
        # If facing wrong direction, rotate first (with smoother rotation)
        if abs(angle_diff) > self.angle_tolerance:
            # Proportional angular control for rotation
            angular_gain = 1.0
            twist.angular.z = angular_gain * self.angular_speed if angle_diff > 0 else -angular_gain * self.angular_speed
            twist.linear.x = 0.0
        else:
            # Move forward with angular correction
            # Use stronger scale so we don't crawl forever
            twist.linear.x = min(self.linear_speed, max(0.8, distance * 1.2))
            twist.angular.z = 0.5 * angle_diff
        
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

