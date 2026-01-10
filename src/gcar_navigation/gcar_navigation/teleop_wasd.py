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
        self.angular_speed = 1.2  # rad/s
        
        # Current pressed keys (for hold-to-move behavior)
        self.keys_pressed = set()
        
        # Settings (will be set in run())
        self.settings = None
        
        # Publish rate (Hz)
        self.publish_rate = 20.0
        
        # Key release timeout (if no key for this long, assume released)
        self.key_timeout = 0.15  # seconds
        self.last_key_time = 0.0
        
        self.get_logger().info('WASD Teleop Node Started')
        self.print_instructions()
        
    def print_instructions(self):
        """Print control instructions."""
        print('\n' + '='*50)
        print('GCAR WASD Teleop Controls - HOLD TO MOVE')
        print('='*50)
        print('Movement (Hold key to move, release to stop):')
        print('  W - Forward (hold to move)')
        print('  S - Backward (hold to move)')
        print('  A - Rotate LEFT (hold to turn)')
        print('  D - Rotate RIGHT (hold to turn)')
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

    def run(self):
        """Main teleop loop with hold-to-move behavior.
        
        Keys are tracked in real-time:
        - Hold key down = robot moves
        - Release key = robot stops
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
                    self.last_key_time = current_time
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
                    
                    # Movement keys - add to pressed keys set
                    elif key_lower in ['w', 's', 'a', 'd']:
                        self.keys_pressed.add(key_lower)
                
                # Check for key timeout (no keys received = all released)
                if current_time - self.last_key_time > self.key_timeout:
                    self.keys_pressed.clear()
                
                # Calculate velocity based on currently pressed keys
                target_linear = 0.0
                target_angular = 0.0
                
                if 'w' in self.keys_pressed:
                    target_linear = self.linear_speed
                elif 's' in self.keys_pressed:
                    target_linear = -self.linear_speed
                
                if 'a' in self.keys_pressed:
                    target_angular = self.angular_speed  # Positive = left
                elif 'd' in self.keys_pressed:
                    target_angular = -self.angular_speed  # Negative = right
                
                # Publish at fixed rate
                if current_time - last_publish_time >= publish_interval:
                    msg = Twist()
                    msg.linear.x = target_linear
                    msg.angular.z = target_angular
                    self.publisher_.publish(msg)
                    last_publish_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error in teleop loop: {e}')
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # Stop the robot
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
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
