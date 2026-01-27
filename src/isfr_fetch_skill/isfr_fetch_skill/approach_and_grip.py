import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf_transformations
from isfr_bot_msgs.msg import GraspSafeObjectArray
from .OdomObjectTracker import OdomObjectTracker

# --- Configuration ---
CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "FOV": 1.57
}

class ApproachGrip(Node):
    def __init__(self):
        super().__init__('approach_grip')
        self.bridge = CvBridge()
        
        # --- Helper Classes ---
        self.tracker = OdomObjectTracker(CAMERA_PARAMS)

        # --- State Machine ---
        # States: "WAIT_FOR_OBJECTS" -> "LOCK_TARGET" -> "TRACK_OBJECT"
        self.state = "WAIT_FOR_OBJECTS"
        self.pending_object_data = None 
        self.current_odom_matrix = None
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.grasp_sub = self.create_subscription(GraspSafeObjectArray, '/vision/grasp_safe_objects', self.grasp_objects_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/isfr/camera_sensor/depth/image', self.depth_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/grasp_track_debug', 10)

        # Oscillation params
        self.yaw_amplitude_rad = math.radians(10.0)
        self.yaw_frequency_hz = 0.2 
        self.start_time = None
        
        self.create_timer(0.05, self.timer_callback)

    # =========================================
    # CALLBACKS
    # =========================================

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        t = msg.pose.pose.position
        T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = [t.x, t.y, t.z]
        self.current_odom_matrix = T

    def grasp_objects_callback(self, msg):
        # Only listen if we are waiting for an object
        if self.state != "WAIT_FOR_OBJECTS" or not msg.objects:
            return
        self.state_wait_for_object(msg)

    def depth_callback(self, msg):
        # get status
        if self.current_odom_matrix is None: return
        T_base_cam = self.get_camera_transform()
        if T_base_cam is None: return
        depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')

        # state machine
        if self.state == "LOCK_TARGET":
            self.state_lock_target(depth_image, T_base_cam)
        elif self.state == "TRACK_OBJECT":
            self.state_track_object(depth_image, T_base_cam)

    def timer_callback(self):
        # Only move if we are actively tracking
        if self.state != "TRACK_OBJECT" or self.start_time is None:
            return
        
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        # Oscillate
        w = 2 * math.pi * self.yaw_frequency_hz
        vel_z = self.yaw_amplitude_rad * w * math.cos(w * t)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.angular.z = vel_z
        self.cmd_pub.publish(twist)
    
    # =========================================
    # STATES
    # =========================================   

    def state_wait_for_object(self, msg):
        # Pick object closest to image center
        target_obj = min(msg.objects, key=lambda o: abs((o.xmin + o.xmax)/2 - CAMERA_PARAMS['width']/2))
        
        self.get_logger().info(f"Target Selected: {target_obj.label}. Transitioning to LOCK_TARGET.")
        
        # Store object data and transition to LOCK state
        # We do NOT track yet, we wait for the next Depth frame to get Z and lock the 3D point.
        self.pending_object_data = target_obj
        self.state = "LOCK_TARGET"

    def state_lock_target(self, depth_image, T_base_cam):
        obj = self.pending_object_data
            
        # Calculate (u, v) based on the bounding box found in the detection step
        bb_width = obj.xmax - obj.xmin
        u_norm = obj.graspline_u + (obj.graspline_width / 2.0)
        
        u = int(obj.xmin + bb_width * u_norm)
        v = int(obj.ymin + (obj.ymax - obj.ymin) * obj.graspline_v)
        
        # Clip
        u = np.clip(u, 0, CAMERA_PARAMS['width'] - 1)
        v = np.clip(v, 0, CAMERA_PARAMS['height'] - 1)
        
        # Sample Z
        z = depth_image[v, u]
        # Try to lock
        success = self.tracker.lock_target(u, v, z, self.current_odom_matrix, T_base_cam)
        
        if success:
            self.get_logger().info(f"Target Locked at (u={u}, v={v}, z={z:.2f})m. Transitioning to TRACK_OBJECT.")
            self.state = "TRACK_OBJECT"
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        else:
            self.get_logger().warn("Failed to get valid depth for target. Retrying or resetting...")
            # Optional: could go back to WAIT_FOR_OBJECTS if this fails repeatedly

    def state_track_object(self, depth_image, T_base_cam):
        # Project stored world point to current image
        uv_pred = self.tracker.get_projected_pixel(self.current_odom_matrix, T_base_cam)
        # Visualization
        debug_img = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)
        if uv_pred:
            u_pred, v_pred = uv_pred
            # Green cross: The tracked position based purely on Odom
            cv2.drawMarker(debug_img, (u_pred, v_pred), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        else:
            self.get_logger().warn("Target out of view (behind camera or invalid projection)")
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))

    # =========================================
    # HELPER
    # =========================================

    def get_camera_transform(self):
        """Helper to get T_base_camera safely"""
        try:
            tf_c = self.tf_buffer.lookup_transform("base_link", "camera_sensor", rclpy.time.Time())
            T = tf_transformations.quaternion_matrix([
                tf_c.transform.rotation.x, tf_c.transform.rotation.y, 
                tf_c.transform.rotation.z, tf_c.transform.rotation.w])
            T[0:3, 3] = [
                tf_c.transform.translation.x, tf_c.transform.translation.y, 
                tf_c.transform.translation.z]
            return T
        except Exception:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ApproachGrip()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()