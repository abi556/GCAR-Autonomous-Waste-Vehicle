#!/usr/bin/env python3
"""Waste detection node using color thresholding."""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
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
        
        # Publisher for detected waste
        self.waste_pub = self.create_publisher(String, '/detected_waste', 10)
        
        # Minimum contour area to consider (filters out noise)
        # Larger values = object must be closer to robot
        self.min_contour_area = 2500
        
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
        self.get_logger().info('Subscribed to: /gcar/camera/image_raw')
        self.get_logger().info('Publishing to: /detected_waste')

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
        
        # Decide label (prioritize larger/closer object)
        label = None
        if red_detected and blue_detected:
            label = 'red_waste' if red_area >= blue_area else 'blue_waste'
        elif red_detected:
            label = 'red_waste'
        elif blue_detected:
            label = 'blue_waste'

        if label is None:
            return

        # De-spam: publish only if changed OR cooldown expired
        if label != self.last_published:
            self.publish_detection(label)
        else:
            # same as last label; rely on cooldown to avoid spamming
            self.publish_detection(label, force_cooldown=True)

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

