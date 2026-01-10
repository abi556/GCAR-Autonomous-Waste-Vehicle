#!/usr/bin/env python3
"""Waste detection node using color thresholding."""

import cv2
import numpy as np
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError


class WasteDetector(Node):
    """Detect waste objects using color thresholding.
    
    Red objects = Hazardous/General Waste
    Blue objects = Recyclable
    """

    def __init__(self):
        super().__init__('waste_detector')
        
        # CV Bridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()
        
        # Subscriber to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/gcar/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Subscriber to robot odometry (to know robot position for bin filtering)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/gcar/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for detected waste
        self.waste_pub = self.create_publisher(String, '/detected_waste', 10)
        
        # Robot's current position (updated by odometry)
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Known bin locations in world frame (x, y)
        # These are the ROADSIDE bins from city.world that robot can access
        self.red_bin_locations = [
            (3.5, 8.0),    # roadside_bin_red_1
            (-3.5, -8.0),  # roadside_bin_red_2
            (10.0, 3.5),   # roadside_bin_red_3
        ]
        self.blue_bin_locations = [
            (3.5, 12.0),   # roadside_bin_blue_1
            (-3.5, -12.0), # roadside_bin_blue_2
            (-10.0, -3.5), # roadside_bin_blue_3
        ]
        
        # Distance threshold to consider detected object as a bin (not waste)
        # If robot is within this distance to a bin, detected color is the bin itself
        self.bin_proximity_threshold = 3.0  # meters
        
        # Minimum contour area to consider (filters out noise)
        # Larger values = object must be closer to robot
        self.min_contour_area = 2500
        
        # Size threshold for noise filtering only
        # Primary filtering is LOCATION-BASED (bins at known, fixed positions)
        # If robot is near bin location → it's the bin
        # If robot is NOT near bin → it's waste (regardless of size!)
        self.min_bin_area = 10000     # pixels - minimum size to be considered a bin at known location
        
        # HSV color ranges for red (red wraps around in HSV, so two ranges)
        # Lower red range (0-10)
        # NOTE: Lower S/V thresholds to work with darker reds in Gazebo lighting
        self.red_lower1 = np.array([0, 60, 50])
        self.red_upper1 = np.array([10, 255, 255])
        # Upper red range (160-180)
        self.red_lower2 = np.array([160, 60, 50])
        self.red_upper2 = np.array([180, 255, 255])
        
        # HSV color range for blue (Gazebo bins are vivid blue)
        # OpenCV HSV hue for blue is ~100-130
        self.blue_lower = np.array([95, 80, 60])
        self.blue_upper = np.array([135, 255, 255])

        # Detection gating: require object to be in front of camera (center-ish + lower half only)
        # This reduces false positives from distant scenery.
        self.center_x_min = 0.30   # fraction of image width (tighter horizontal window)
        self.center_x_max = 0.70
        self.center_y_min = 0.40   # fraction of image height (only lower 60% - closer objects)

        # Publish only on change (plus cooldown) to avoid spamming same label
        self.last_published = None
        
        # Cooldown to prevent spam publishing
        self.last_detection_time = self.get_clock().now()
        self.detection_cooldown = 1.0  # seconds
        
        self.get_logger().info('Waste Detector Node Started')
        self.get_logger().info('Subscribed to: /gcar/camera/image_raw, /gcar/odom')
        self.get_logger().info('Publishing to: /detected_waste')
        self.get_logger().info(f'Bin proximity threshold: {self.bin_proximity_threshold}m')

    def odom_callback(self, msg):
        """Update robot position from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def is_near_bin(self, color):
        """Check if robot is near any bin of the specified color.
        
        Args:
            color: 'red' or 'blue'
            
        Returns:
            True if robot is within bin_proximity_threshold of a bin
        """
        bin_locations = self.red_bin_locations if color == 'red' else self.blue_bin_locations
        
        for bin_x, bin_y in bin_locations:
            distance = math.sqrt((self.robot_x - bin_x)**2 + (self.robot_y - bin_y)**2)
            if distance < self.bin_proximity_threshold:
                return True
        return False

    def image_callback(self, msg):
        """Process incoming camera images for waste detection."""
        try:
            # Convert ROS Image to OpenCV format (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return
        
        # Convert BGR to HSV for color detection
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        h, w = hsv_image.shape[:2]

        # Detect red objects
        red_detected, red_area = self.detect_color(
            hsv_image,
            [(self.red_lower1, self.red_upper1),
             (self.red_lower2, self.red_upper2)],
            w=w,
            h=h,
        )

        # Detect blue objects (recyclable)
        blue_detected, blue_area = self.detect_color(
            hsv_image,
            [(self.blue_lower, self.blue_upper)],
            w=w,
            h=h,
        )
        
        # Check cooldown
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_detection_time).nanoseconds / 1e9
        
        if time_diff < self.detection_cooldown:
            return
        
        # Classify detected objects as bins or waste
        # Strategy: Size-based + location-based filtering
        label = None
        
        if red_detected and blue_detected:
            # Both colors detected - prioritize larger one
            if red_area >= blue_area:
                label = self.classify_object('red', red_area)
            else:
                label = self.classify_object('blue', blue_area)
        elif red_detected:
            label = self.classify_object('red', red_area)
        elif blue_detected:
            label = self.classify_object('blue', blue_area)

        if label is None:
            return

        # Only publish WASTE detections, ignore bins
        if label.endswith('_waste'):
            # De-spam: publish only if changed OR cooldown expired
            if label != self.last_published:
                self.publish_detection(label)
            else:
                # same as last label; rely on cooldown to avoid spamming
                self.publish_detection(label, force_cooldown=True)

    def classify_object(self, color, area):
        """Classify detected colored object as bin or waste.
        
        Strategy: LOCATION-BASED filtering ONLY (size only filters noise).
        - Bins are at KNOWN, FIXED locations (hardcoded in is_near_bin)
        - If robot is near bin location → it's the bin (ignore)
        - If robot is NOT near bin → it's waste (detect!) regardless of size
        
        Args:
            color: 'red' or 'blue'
            area: contour area in pixels
            
        Returns:
            'red_waste', 'blue_waste', 'red_bin', 'blue_bin', or None
        """
        # PRIMARY check: Is robot near a known bin of this color?
        near_bin = self.is_near_bin(color)
        
        # Secondary check: Is object large enough (filters noise only)
        is_large_enough = area > self.min_bin_area
        
        # Debug logging
        self.get_logger().info(
            f'Classify {color}: area={area:.0f}px, robot_pos=({self.robot_x:.2f},{self.robot_y:.2f}), '
            f'near_bin={near_bin}, large={is_large_enough}'
        )
        
        # SIMPLE LOGIC:
        # 1. Near bin + large → it's the bin (ignore)
        # 2. NOT near bin → it's waste! (regardless of size)
        # 3. Near bin + small → noise (ignore)
        
        if near_bin and is_large_enough:
            # Robot is near known bin location, sees large colored object = the bin itself
            self.get_logger().info(f'→ Classified as {color}_bin (near known bin location)')
            return f'{color}_bin'
        elif not near_bin:
            # Robot is NOT near any bin = WASTE (even if huge, it's just close-up waste!)
            self.get_logger().info(f'→ Classified as {color}_waste (not near any bin)')
            return f'{color}_waste'
        else:
            # Near bin but object too small (probably noise)
            self.get_logger().info(f'→ Ignored (near bin but too small, likely noise)')
            return None

    def detect_color(self, hsv_image, color_ranges, w, h):
        """Detect objects of specified color(s).
        
        Args:
            hsv_image: Image in HSV color space
            color_ranges: List of (lower, upper) HSV range tuples
            
        Returns:
            (detected: bool, max_area: float)
        """
        # Create combined mask for all color ranges
        combined_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        for lower, upper in color_ranges:
            mask = cv2.inRange(hsv_image, lower, upper)
            combined_mask = cv2.bitwise_or(combined_mask, mask)
        
        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.erode(combined_mask, kernel, iterations=1)
        combined_mask = cv2.dilate(combined_mask, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(
            combined_mask, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Pick best contour that also passes "in-front" gating
        max_area = 0.0
        best_ok = False

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_contour_area:
                continue

            x, y, cw, ch = cv2.boundingRect(contour)
            cx = x + cw / 2.0
            cy = y + ch / 2.0

            # Gate by centroid position (object should be roughly in front)
            if not (self.center_x_min * w <= cx <= self.center_x_max * w):
                continue
            if not (cy >= self.center_y_min * h):
                continue

            # Gate by size relative to image (avoid tiny distant detections)
            # Require at least 1.5% of image to ensure object is close enough
            if area < 0.015 * (w * h):  # at least 1.5% of image pixels
                continue

            if area > max_area:
                max_area = area
                best_ok = True

        return best_ok, max_area

    def publish_detection(self, waste_type, force_cooldown=False):
        """Publish detected waste type."""
        # Cooldown check (unless label changed)
        if not force_cooldown:
            pass
        else:
            current_time = self.get_clock().now()
            time_diff = (current_time - self.last_detection_time).nanoseconds / 1e9
            if time_diff < self.detection_cooldown:
                return

        msg = String()
        msg.data = waste_type
        self.waste_pub.publish(msg)
        self.last_detection_time = self.get_clock().now()
        self.last_published = waste_type
        self.get_logger().info(f'Detected: {waste_type}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = WasteDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

