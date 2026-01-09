#!/usr/bin/env python3
"""WASD keyboard teleop for GCAR robot."""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopWASD(Node):
    """WASD keyboard teleop node for GCAR."""

    def __init__(self):
        super().__init__('teleop_wasd')
        
        # Publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/gcar/cmd_vel', 10)
        
        # Speed parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.8  # rad/s (reduced for smoother rotation)
        
        # Settings (will be set in run())
        self.settings = None
        
        self.get_logger().info('WASD Teleop Node Started')
        self.print_instructions()
        
    def print_instructions(self):
        """Print control instructions."""
        print('\n' + '='*50)
        print('GCAR WASD Teleop Controls')
        print('='*50)
        print('Movement:')
        print('  W - Move forward')
        print('  S - Move backward')
        print('  A - Turn left (rotate counter-clockwise)')
        print('  D - Turn right (rotate clockwise)')
        print('  W+A - Move forward while turning left')
        print('  W+D - Move forward while turning right')
        print('  Q - Stop')
        print('\nSpeed Control:')
        print('  + / = - Increase linear speed by 10%')
        print('  - / _ - Decrease linear speed by 10%')
        print('  [ - Increase angular speed by 10%')
        print('  ] - Decrease angular speed by 10%')
        print('\n  CTRL-C to quit')
        print('='*50)
        print(f'Current speeds: Linear={self.linear_speed:.2f} m/s, Angular={self.angular_speed:.2f} rad/s')
        print('='*50 + '\n')

    def get_key(self):
        """Get a single keypress from stdin."""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def run(self):
        """Main teleop loop."""
        # Get terminal settings before modifying
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key is None:
                    continue
                
                # Handle key presses
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                
                key_lower = key.lower()
                
                # Speed adjustment keys (handle first, then continue)
                if key == '+' or key == '=':
                    self.linear_speed = min(2.0, self.linear_speed * 1.1)
                    print(f'\rLinear speed: {self.linear_speed:.2f} m/s', end='', flush=True)
                    continue
                elif key == '-' or key == '_':
                    self.linear_speed = max(0.1, self.linear_speed * 0.9)
                    print(f'\rLinear speed: {self.linear_speed:.2f} m/s', end='', flush=True)
                    continue
                elif key == '[':
                    self.angular_speed = min(3.0, self.angular_speed * 1.1)
                    print(f'\rAngular speed: {self.angular_speed:.2f} rad/s', end='', flush=True)
                    continue
                elif key == ']':
                    self.angular_speed = max(0.1, self.angular_speed * 0.9)
                    print(f'\rAngular speed: {self.angular_speed:.2f} rad/s', end='', flush=True)
                    continue
                elif key == '\x03':  # CTRL-C
                    break
                
                # Movement keys - WASD
                if key_lower == 'w':
                    msg.linear.x = self.linear_speed
                elif key_lower == 's':
                    msg.linear.x = -self.linear_speed
                
                if key_lower == 'a':
                    msg.angular.z = self.angular_speed
                elif key_lower == 'd':
                    msg.angular.z = -self.angular_speed
                
                # Q to stop (explicitly zero)
                if key_lower == 'q':
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                
                # Publish the command
                self.publisher_.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in teleop loop: {e}')
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # Stop the robot
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
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

