import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image

class DepthSentry(Node):
    def __init__(self):
        super().__init__('depth_sentry')

        # 1. Subscribe to the specific Depth Topic
        self.subscription = self.create_subscription(
            Image,
            '/isfr/camera_sensor/depth/image',
            self.listener_callback,
            10)

        # 2. Publisher: Alarm signal
        self.alarm_publisher = self.create_publisher(Bool, '/isfr/security/alarm_triggered', 10)

        # Debug publisher for RViz
        self.mask_publisher = self.create_publisher(Image, '/isfr/security/depth_mask', 10)

        self.bridge = CvBridge()
        self.prev_frame = None

        self.get_logger().info("Depth Sentry Initialized. Watching for movement in depth field...")

    def listener_callback(self, msg):
        try:
            # Convert ROS Depth message to OpenCV (usually 32FC1 - Float 32-bit single channel)
            # 'passthrough' keeps the original encoding (meters/millimeters)
            current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # 1. Handle NaNs (Depth cameras often have "invalid" pixels represented as NaN)
            # We replace NaNs with 0 to prevent errors during math operations
            current_frame = np.nan_to_num(current_frame, nan=0.0)

            # Initialize previous frame if empty
            if self.prev_frame is None:
                self.prev_frame = current_frame
                return

            # --- DEPTH MOTION LOGIC ---
            
            # 2. Calculate absolute difference in DEPTH (Distance change)
            # This detects if an object physically moved closer or further
            diff = cv2.absdiff(current_frame, self.prev_frame)

            # 3. Thresholding
            # We look for changes > 0.1 meters (adjust this value based on sensitivity needs)
            # Since the image is float, 0.1 usually means 10cm or 100mm depending on camera config
            threshold_value = 0.5  # Example: 0.5 unit change triggers motion
            _, thresh = cv2.threshold(diff, threshold_value, 255, cv2.THRESH_BINARY)
            
            # Convert to uint8 for contour detection (required by OpenCV)
            thresh = thresh.astype(np.uint8)

            # 4. Clean up noise
            thresh = cv2.dilate(thresh, None, iterations=2)

            # -- PUBLISH MASK TO TOPIC --
            mask_msg = self.bridge.cv2_to_imgmsg(thresh, encoding="mono8")
            mask_msg.header = msg.header
            self.mask_publisher.publish(mask_msg)

            # 5. Check for Motion Area
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            motion_detected = False
            for contour in contours:
                if cv2.contourArea(contour) < 500:  # Ignore small speckles
                    continue
                motion_detected = True
                break  # If we found one big movement, that's enough

            # --- ALARM ---
            alarm_msg = Bool()
            alarm_msg.data = motion_detected
            self.alarm_publisher.publish(alarm_msg)

            # if motion_detected:
            #     self.get_logger().warn('INTRUDER ALERT: Depth changes detected!')

            # Update previous frame
            self.prev_frame = current_frame

        except Exception as e:
            self.get_logger().error(f'Frame processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthSentry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()