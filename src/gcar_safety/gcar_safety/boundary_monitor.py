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
        
        # Out of bounds state
        self.is_out_of_bounds = False
        
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
        
        # Timer for periodic boundary checks (10 Hz - fast enough to override teleop)
        self.timer = self.create_timer(0.1, self.check_boundaries)
        
        # High-frequency emergency stop timer (50 Hz - only active when out of bounds)
        self.emergency_timer = None
        
        self.get_logger().info('Boundary Monitor Node Started')
        self.get_logger().info(f'Operational Area: X[{self.x_min}, {self.x_max}], Y[{self.y_min}, {self.y_max}]')
        self.get_logger().info(f'Warning margin: {self.warning_margin}m')
        self.get_logger().info(f'Emergency stop: {"ENABLED (50 Hz override)" if self.emergency_stop else "DISABLED"}')

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
            
            # Activate high-frequency emergency stop if not already active
            if not self.is_out_of_bounds:
                self.is_out_of_bounds = True
                self.get_logger().error(warning_msg)
                self.get_logger().error('Activating high-frequency emergency stop (50 Hz)')
                
                # Start high-frequency timer to override teleop (50 Hz >> teleop's 20 Hz)
                if self.emergency_stop:
                    self.emergency_timer = self.create_timer(0.02, self.emergency_stop_callback)
            
            self.last_warning = warning_msg
            
            # Publish warning (every cycle while out of bounds)
            msg = String()
            msg.data = warning_msg
            self.warning_pub.publish(msg)
            
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
            # Deactivate emergency timer if robot is back in bounds
            if self.is_out_of_bounds:
                self.is_out_of_bounds = False
                self.get_logger().info('Back in safe zone - deactivating emergency stop')
                
                # Stop the high-frequency emergency timer
                if self.emergency_timer is not None:
                    self.emergency_timer.cancel()
                    self.emergency_timer = None
            
            # Clear warning if back in safe zone
            if self.last_warning is not None:
                self.last_warning = None

    def emergency_stop_callback(self):
        """High-frequency emergency stop callback (50 Hz).
        
        This publishes stop commands faster than teleop (20 Hz) to override it.
        Only active when robot is out of bounds.
        """
        self.publish_stop()
    
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
