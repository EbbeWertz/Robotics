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

CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "FOV": 1.57
}

def invert_transform(T):
    R_inv = T[0:3, 0:3].T
    t_inv = -R_inv @ T[0:3, 3]
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = t_inv
    return T_inv

class ApproachGrip(Node):
    def __init__(self):
        super().__init__('approach_grip')
        self.bridge = CvBridge()

        # --- State ---
        self.state = "WAIT_FOR_OBJECTS"
        self.target_object = None
        self.initial_point_world = None 

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

        # Intrinsics
        self.fx = CAMERA_PARAMS['width'] / (2 * math.tan(CAMERA_PARAMS['FOV'] / 2))
        self.fy = self.fx
        self.cx = CAMERA_PARAMS['width'] / 2
        self.cy = CAMERA_PARAMS['height'] / 2

        # Oscillation params
        self.yaw_amplitude_rad = math.radians(10.0)
        self.yaw_frequency_hz = 0.2 
        self.start_time = None
        
        self.create_timer(0.05, self.timer_callback)

    def odom_callback(self, msg):
        # Convert Odometry msg to 4x4 Matrix
        q = msg.pose.pose.orientation
        t = msg.pose.pose.position
        T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = [t.x, t.y, t.z]
        self.current_odom_matrix = T

    def grasp_objects_callback(self, msg):
        if self.state != "WAIT_FOR_OBJECTS" or not msg.objects:
            return
        # Pick object closest to image center
        self.target_object = min(msg.objects, key=lambda o: abs((o.xmin + o.xmax)/2 - self.cx))
        self.state = "TRACK_OBJECT"
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Tracking: {self.target_object.label}")

    def depth_callback(self, msg):
        if self.state != "TRACK_OBJECT" or self.current_odom_matrix is None:
            return

        try:
            # Crucial: Look up the transform at the time of the image/odom
            # We use 'camera_optical_link' because projection math assumes Z-forward
            tf_c = self.tf_buffer.lookup_transform("base_link", "camera_sensor", rclpy.time.Time())
            base_to_camera = tf_transformations.quaternion_matrix([tf_c.transform.rotation.x, tf_c.transform.rotation.y, tf_c.transform.rotation.z, tf_c.transform.rotation.w])
            base_to_camera[0:3, 3] = [tf_c.transform.translation.x, tf_c.transform.translation.y, tf_c.transform.translation.z]
        except Exception as e:
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        
        # Calculate grab pixel (u, v)
        obj = self.target_object
        bb_width = obj.xmax - obj.xmin
        u_normalized_center = obj.graspline_u + (obj.graspline_width / 2.0)
        u = int(obj.xmin + bb_width * u_normalized_center)
        v = int(obj.ymin + (obj.ymax - obj.ymin) * obj.graspline_v)
        
        # Ensure u, v are in bounds
        u = np.clip(u, 0, CAMERA_PARAMS['width'] - 1)
        v = np.clip(v, 0, CAMERA_PARAMS['height'] - 1)
        z = depth_image[v, u]

        if not np.isfinite(z) or z <= 0:
            return

        # 1. Project to Camera Frame (P_cam)
        P_cam = np.array([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z, 1.0])

        # 2. Store first sighting in World Frame
        if self.initial_point_world is None:
            # World = Odom * Base_to_Cam * P_cam
            self.initial_point_world = self.current_odom_matrix @ base_to_camera @ P_cam
            return

        # 3. Project World Point back to current Camera Frame
        # P_current_cam = inv(Odom_now * Base_to_Cam) * P_world
        P_base_now = invert_transform(self.current_odom_matrix) @ self.initial_point_world
        P_cam_now = invert_transform(base_to_camera) @ P_base_now
        Xc, Yc, Zc, _ = P_cam_now
        u_pred = int(self.fx * Xc / Zc + self.cx)
        v_pred = int(self.fy * Yc / Zc + self.cy)
        
        # --- Visualization ---
        debug_img = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)
        # Green cross: Predicted position based on Odom
        cv2.drawMarker(debug_img, (u_pred, v_pred), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        # Red circle: Actual detection point
        cv2.circle(debug_img, (u, v), 5, (0, 0, 255), -1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
    
    def timer_callback(self):
        if self.state != "TRACK_OBJECT" or self.start_time is None:
            return
        
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        # Velocity is the derivative of position: 
        # pos = A * sin(2pi * f * t) -> vel = A * 2pi * f * cos(2pi * f * t)
        w = 2 * math.pi * self.yaw_frequency_hz
        vel_z = self.yaw_amplitude_rad * w * math.cos(w * t)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.angular.z = vel_z
        self.cmd_pub.publish(twist)

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