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
from isfr_bot_msgs.msg import GraspSafeObjectArray, GraspSafeObject

CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "FOV": 1.57
}


def transform_matrix_from_odom(msg: Odometry):
    """Return 4x4 homogeneous transform from odometry message."""
    q = msg.pose.pose.orientation
    t = msg.pose.pose.position

    trans = np.array([t.x, t.y, t.z])
    rot = [q.x, q.y, q.z, q.w]
    R = tf_transformations.quaternion_matrix(rot)  # 4x4
    R[0:3, 3] = trans
    return R


def invert_transform(T):
    """Invert a 4x4 homogeneous transform."""
    R_inv = T[0:3, 0:3].T
    t_inv = -R_inv @ T[0:3, 3]
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = t_inv
    return T_inv


def tf_to_matrix(tf_msg):
    """Convert geometry_msgs/TransformStamped to 4x4 numpy matrix."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0:3, 3] = [t.x, t.y, t.z]
    return T


class ApproachGrip(Node):
    def __init__(self):
        super().__init__('approach_grip')
        self.bridge = CvBridge()

        # --- State ---
        self.state = "WAIT_FOR_OBJECTS"
        self.target_object = None
        self.initial_cam_point_world = None  # 3D point in world frame

        # Latest depth
        self.latest_depth = None

        # --- Odometry / camera offset ---
        self.current_odom = None
        self.base_to_camera = None  # 4x4 matrix

        # --- TF2 buffer + listener ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        self.grasp_sub = self.create_subscription(
            GraspSafeObjectArray,
            '/vision/grasp_safe_objects',
            self.grasp_objects_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/isfr/camera_sensor/depth/image',
            self.depth_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            50
        )

        # --- Publisher for diffdrive oscillation ---
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
        self.yaw_amplitude_deg = 5.0
        self.yaw_frequency_hz = 0.05  # very slow oscillation
        self.start_time = None
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Debug image publisher
        self.debug_pub = self.create_publisher(Image, '/vision/grasp_track_debug', 10)

        # Camera intrinsics
        self.cx = CAMERA_PARAMS['width'] / 2
        self.cy = CAMERA_PARAMS['height'] / 2
        self.fx = CAMERA_PARAMS['width'] / (2 * math.tan(CAMERA_PARAMS['FOV'] / 2))
        self.fy = self.fx  # assume square pixels

        # --- Try to fetch base->camera transform once ---
        self.get_logger().info("Waiting for base_link -> camera_sensor TF...")
        self.create_timer(1.0, self.init_base_to_camera)

    def init_base_to_camera(self):
        if self.base_to_camera is not None:
            return
        try:
            tf_msg = self.tf_buffer.lookup_transform("base_link", "camera_sensor", rclpy.time.Time())
            self.base_to_camera = tf_to_matrix(tf_msg)
            self.get_logger().info("Base->Camera transform initialized.")
        except Exception as e:
            self.get_logger().warn(f"Base->Camera TF not ready: {e}")

    # -----------------------
    # Grasp object selection
    # -----------------------
    def grasp_objects_callback(self, msg: GraspSafeObjectArray):
        if self.state != "WAIT_FOR_OBJECTS":
            return
        if not msg.objects:
            return

        center_u = CAMERA_PARAMS['width'] / 2
        chosen_obj = min(msg.objects, key=lambda o: abs((o.xmin + o.xmax)/2 - center_u))

        if chosen_obj:
            self.target_object = chosen_obj
            self.get_logger().info(f"Target chosen: {chosen_obj.label}")
            self.state = "TRACK_OBJECT"
            self.start_time = self.get_clock().now().nanoseconds / 1e9

    # -----------------------
    # Odometry update
    # -----------------------
    def odom_callback(self, msg: Odometry):
        self.current_odom = transform_matrix_from_odom(msg)

    # -----------------------
    # Depth callback
    # -----------------------
    def depth_callback(self, msg: Image):
        if self.state != "TRACK_OBJECT" or self.target_object is None or self.current_odom is None:
            return
        if self.base_to_camera is None:
            return  # wait for TF

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.latest_depth = depth_image

        obj = self.target_object
        x1, y1, x2, y2 = map(int, [obj.xmin, obj.ymin, obj.xmax, obj.ymax])
        bb_width = x2 - x1
        bb_height = y2 - y1

        grab_y = int(y1 + obj.graspline_v * bb_height)
        grab_x = int(x1 + obj.graspline_u * bb_width) + int(obj.graspline_width * bb_width)//2

        z = depth_image[grab_y, grab_x]
        if not np.isfinite(z) or z <= 0:
            return

        # --- Point in camera frame ---
        P_cam = np.array([
            (grab_x - self.cx) * z / self.fx,
            (grab_y - self.cy) * z / self.fy,
            z,
            1.0
        ])

        # --- First frame: save initial world point ---
        if self.initial_cam_point_world is None:
            self.initial_cam_point_world = self.current_odom @ self.base_to_camera @ P_cam
            self.get_logger().info("Initial 3D point in world saved.")
            return

        # --- Compute current camera transform in world frame ---
        camera_world = self.current_odom @ self.base_to_camera
        world_to_camera = invert_transform(camera_world)

        # --- Transform initial point into current camera frame ---
        P_cur_cam = world_to_camera @ self.initial_cam_point_world
        Xc, Yc, Zc, _ = P_cur_cam
        if Zc <= 0:
            return

        # --- Project back to pixels ---
        u_pred = int(self.fx * Xc / Zc + self.cx)
        v_pred = int(self.fy * Yc / Zc + self.cy)

        delta_u = u_pred - grab_x
        self.get_logger().info(f"Z: {z:.3f} m | Δu: {delta_u} px")

        # --- Visualization ---
        debug_img = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)
        px = max(0, min(CAMERA_PARAMS['width']-1, u_pred))
        cv2.line(debug_img, (px, 0), (px, CAMERA_PARAMS['height']-1), (0, 255, 0), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))

    # -----------------------
    # Diffdrive oscillation
    # -----------------------
    def timer_callback(self):
        if self.state != "TRACK_OBJECT":
            return
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        yaw_rad = math.radians(self.yaw_amplitude_deg) * math.sin(2 * math.pi * self.yaw_frequency_hz * t)
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.angular.z = yaw_rad
        twist.twist.linear.x = 0.0
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
