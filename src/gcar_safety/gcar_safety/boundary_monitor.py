#!/usr/bin/env python3
"""Boundary monitor node to keep robot within safe operational area."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class BoundaryMonitor(Node):
    """Monitor robot position and enforce operational boundaries.
    
    Prevents the robot from wandering outside the city limits by:
    - Publishing warnings when approaching boundaries
    - Optionally stopping the robot if it exceeds boundaries
    """

    def __init__(self):
        super().__init__('boundary_monitor')
        
        # Declare parameters
        self.declare_parameter('x_min', -32.0)
        self.declare_parameter('x_max', 32.0)
        self.declare_parameter('y_min', -32.0)
        self.declare_parameter('y_max', 32.0)
        self.declare_parameter('warning_margin', 5.0)  # meters from boundary to start warning
        self.declare_parameter('emergency_stop', True)  # stop robot if out of bounds
        
        # Get parameters
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.warning_margin = self.get_parameter('warning_margin').value
        self.emergency_stop = self.get_parameter('emergency_stop').value
        
        # Current robot position
        self.current_x = 0.0
        self.current_y = 0.0
        self.position_received = False
        
        # Warning state tracking
        self.last_warning = None
        
        # Subscriber to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/gcar/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for warnings
        self.warning_pub = self.create_publisher(String, '/gcar/safety/boundary_warning', 10)
        
        # Publisher for emergency stop (publishes zero velocity)
        self.stop_pub = self.create_publisher(Twist, '/gcar/cmd_vel', 10)
        
        # Timer for periodic boundary checks (5 Hz)
        self.timer = self.create_timer(0.2, self.check_boundaries)
        
        self.get_logger().info('Boundary Monitor Node Started')
        self.get_logger().info(f'Operational Area: X[{self.x_min}, {self.x_max}], Y[{self.y_min}, {self.y_max}]')
        self.get_logger().info(f'Warning margin: {self.warning_margin}m')
        self.get_logger().info(f'Emergency stop: {"ENABLED" if self.emergency_stop else "DISABLED"}')

    def odom_callback(self, msg):
        """Update current robot position from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.position_received = True

    def check_boundaries(self):
        """Check if robot is within safe boundaries."""
        if not self.position_received:
            return
        
        # Calculate distances to each boundary
        dist_to_x_min = self.current_x - self.x_min
        dist_to_x_max = self.x_max - self.current_x
        dist_to_y_min = self.current_y - self.y_min
        dist_to_y_max = self.y_max - self.current_y
        
        # Find closest boundary
        min_dist = min(dist_to_x_min, dist_to_x_max, dist_to_y_min, dist_to_y_max)
        
        # Determine which boundary is closest
        boundary_name = ""
        if min_dist == dist_to_x_min:
            boundary_name = "WEST"
        elif min_dist == dist_to_x_max:
            boundary_name = "EAST"
        elif min_dist == dist_to_y_min:
            boundary_name = "SOUTH"
        else:
            boundary_name = "NORTH"
        
        # Check if out of bounds (emergency)
        if (self.current_x < self.x_min or self.current_x > self.x_max or
            self.current_y < self.y_min or self.current_y > self.y_max):
            
            warning_msg = f"OUT OF BOUNDS! Position: ({self.current_x:.2f}, {self.current_y:.2f})"
            self.get_logger().error(warning_msg)
            
            # Publish warning
            msg = String()
            msg.data = warning_msg
            self.warning_pub.publish(msg)
            
            # Emergency stop if enabled
            if self.emergency_stop:
                self.publish_stop()
                self.get_logger().warn('Emergency stop issued!')
            
            return
        
        # Check if approaching boundary (warning zone)
        if min_dist < self.warning_margin:
            warning_msg = f"Approaching {boundary_name} boundary! Distance: {min_dist:.2f}m"
            
            # Only log if warning changed (avoid spam)
            if self.last_warning != warning_msg:
                self.get_logger().warn(warning_msg)
                self.last_warning = warning_msg
                
                # Publish warning
                msg = String()
                msg.data = warning_msg
                self.warning_pub.publish(msg)
        else:
            # Clear warning if back in safe zone
            if self.last_warning is not None:
                self.get_logger().info('Back in safe zone')
                self.last_warning = None

    def publish_stop(self):
        """Publish zero velocity to stop the robot."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.stop_pub.publish(stop_msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = BoundaryMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
