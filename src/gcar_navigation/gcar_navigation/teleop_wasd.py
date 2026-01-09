#!/usr/bin/env python3
"""WASD keyboard teleop for GCAR robot with continuous command publishing."""

import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopWASD(Node):
    """WASD keyboard teleop node for GCAR with continuous publishing."""

    def __init__(self):
        super().__init__('teleop_wasd')
        
        # Publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/gcar/cmd_vel', 10)
        
        # Speed parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.2  # rad/s (increased for better rotation)
        
        # Current velocity state
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Settings (will be set in run())
        self.settings = None
        
        # Publish rate (Hz)
        self.publish_rate = 20.0  # Increased rate
        
        self.get_logger().info('WASD Teleop Node Started')
        self.print_instructions()
        
    def print_instructions(self):
        """Print control instructions."""
        print('\n' + '='*50)
        print('GCAR WASD Teleop Controls')
        print('='*50)
        print('Movement (HOLD the key):')
        print('  W - Forward')
        print('  S - Backward')
        print('  A - Rotate LEFT')
        print('  D - Rotate RIGHT')
        print('  X - Stop immediately')
        print('\nSpeed Control:')
        print('  + / = - Increase linear speed')
        print('  - / _ - Decrease linear speed')
        print('  ] - Increase angular speed')
        print('  [ - Decrease angular speed')
        print('\n  CTRL-C to quit')
        print('='*50)
        print(f'Speeds: Linear={self.linear_speed:.2f} m/s, Angular={self.angular_speed:.2f} rad/s')
        print('='*50 + '\n')

    def get_key(self, timeout=0.05):
        """Get a single keypress from stdin with timeout."""
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None

    def publish_velocity(self):
        """Publish current velocity."""
        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular
        self.publisher_.publish(msg)

    def run(self):
        """Main teleop loop with continuous publishing.

        Note: Key-repeat timing varies by OS/terminal. To make rotation reliable,
        we *latch* the last command until you change it or press X to stop.
        """
        # Get terminal settings before modifying
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        
        last_publish_time = time.time()
        publish_interval = 1.0 / self.publish_rate
        
        try:
            while rclpy.ok():
                # Check for keypress (non-blocking)
                key = self.get_key(timeout=0.02)
                
                current_time = time.time()
                
                if key is not None:
                    key_lower = key.lower()
                    
                    # CTRL-C to exit
                    if key == '\x03':
                        break
                    
                    # Speed adjustment keys
                    if key == '+' or key == '=':
                        self.linear_speed = min(2.0, self.linear_speed + 0.1)
                        print(f'\rLinear: {self.linear_speed:.2f} m/s   ', end='', flush=True)
                    elif key == '-' or key == '_':
                        self.linear_speed = max(0.1, self.linear_speed - 0.1)
                        print(f'\rLinear: {self.linear_speed:.2f} m/s   ', end='', flush=True)
                    elif key == ']':
                        self.angular_speed = min(3.0, self.angular_speed + 0.1)
                        print(f'\rAngular: {self.angular_speed:.2f} rad/s   ', end='', flush=True)
                    elif key == '[':
                        self.angular_speed = max(0.2, self.angular_speed - 0.1)
                        print(f'\rAngular: {self.angular_speed:.2f} rad/s   ', end='', flush=True)
                    
                    # Movement keys (latched until another command / stop)
                    elif key_lower == 'w':
                        self.target_linear = self.linear_speed
                    elif key_lower == 's':
                        self.target_linear = -self.linear_speed
                    elif key_lower == 'a':
                        self.target_angular = self.angular_speed  # Positive = left
                    elif key_lower == 'd':
                        self.target_angular = -self.angular_speed  # Negative = right
                    elif key_lower == 'x':
                        self.target_linear = 0.0
                        self.target_angular = 0.0
                        print('\rSTOPPED                    ', end='', flush=True)
                
                # Publish at fixed rate
                if current_time - last_publish_time >= publish_interval:
                    self.publish_velocity()
                    last_publish_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error in teleop loop: {e}')
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # Stop the robot
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.publish_velocity()
            print('\nTeleop stopped. Robot halted.')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = TeleopWASD()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
