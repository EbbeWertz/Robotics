import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from isfr_bot_msgs.msg import GraspSafeObjectArray, GraspSafeObject

CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "FOV": 1.57
}

class ApproachGrip(Node):
    def __init__(self):
        super().__init__('approach_grip')
        self.bridge = CvBridge()

        # --- State ---
        self.state = "WAIT_FOR_OBJECTS"
        self.target_object = None
        self.initial_yaw = None
        self.current_yaw = None
        self.initial_pos = None
        self.current_pos = None
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Latest depth
        self.latest_depth = None

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
            10
        )

        # --- Publisher for diffdrive oscillation ---
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
        self.yaw_amplitude_deg = 5.0
        self.yaw_frequency_hz = 0.05  # very slow oscillation
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Debug image publisher
        self.debug_pub = self.create_publisher(Image, '/vision/grasp_track_debug', 10)

        # Camera intrinsics
        self.cx = CAMERA_PARAMS['width'] / 2
        self.cy = CAMERA_PARAMS['height'] / 2
        self.fx = CAMERA_PARAMS['width'] / (2 * math.tan(CAMERA_PARAMS['FOV'] / 2))
        self.fy = self.fx  # assume square pixels

    # -----------------------
    # State 1: wait & choose target
    # -----------------------
    def grasp_objects_callback(self, msg: GraspSafeObjectArray):
        if self.state != "WAIT_FOR_OBJECTS":
            return
        if not msg.objects:
            return

        # Choose object closest to horizontal center
        center_u = CAMERA_PARAMS['width'] / 2
        min_dist = float('inf')
        chosen_obj = None
        for obj in msg.objects:
            x1 = obj.xmin
            x2 = obj.xmax
            obj_center_u = (x1 + x2) / 2
            dist = abs(obj_center_u - center_u)
            if dist < min_dist:
                min_dist = dist
                chosen_obj = obj

        if chosen_obj:
            self.target_object = chosen_obj
            self.state = "TRACK_OBJECT"
            self.get_logger().info(f"Target object chosen: {chosen_obj.label}")

    # -----------------------
    # Odometry updates
    # -----------------------
    def odom_callback(self, msg: Odometry):
        # Extract yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = yaw

        pos = msg.pose.pose.position
        self.current_pos = np.array([pos.x, pos.y, pos.z])

        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.initial_pos = self.current_pos.copy()

    # -----------------------
    # Depth callback: full 3D predictive tracking
    # -----------------------
    def depth_callback(self, msg: Image):
        if self.state != "TRACK_OBJECT" or self.target_object is None:
            return
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.latest_depth = depth_image

            obj = self.target_object
            x1, y1, x2, y2 = map(int, [obj.xmin, obj.ymin, obj.xmax, obj.ymax])
            bb_width = x2 - x1
            bb_height = y2 - y1
            grab_y_orig = int(y1 + obj.graspline_v * bb_height)
            grab_x_start = int(x1 + obj.graspline_u * bb_width)
            grab_width = int(obj.graspline_width * bb_width)
            grab_x_center = grab_x_start + grab_width // 2

            # --- Depth at grabline center ---
            z = depth_image[grab_y_orig, grab_x_center]
            if not np.isfinite(z) or z <= 0:
                return

            # --- Compute yaw relative to initial ---
            if self.initial_yaw is None or self.current_yaw is None:
                yaw_delta = 0.0
            else:
                yaw_delta = self.current_yaw - self.initial_yaw

            # --- Lateral displacement in meters at depth z ---
            lateral_disp_m = math.tan(yaw_delta) * z

            # --- Convert displacement to pixels ---
            pixels_per_meter = CAMERA_PARAMS['width'] / (2 * math.tan(CAMERA_PARAMS['FOV']/2) * z)
            delta_u = int(lateral_disp_m * pixels_per_meter)

            # Predicted column in image
            predicted_x = grab_x_center + delta_u
            predicted_x = max(0, min(CAMERA_PARAMS['width']-1, predicted_x))

            # --- Logging ---
            self.get_logger().info(
                f"Yaw delta: {math.degrees(yaw_delta):.2f} deg | "
                f"Distance: {z:.3f} m | "
                f"Lateral displacement: {lateral_disp_m:.3f} m | "
                f"Pixel shift: {delta_u} px"
            )

            # --- Visualize vertical line at predicted column ---
            debug_img = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
            debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)
            cv2.line(debug_img, (predicted_x, 0), (predicted_x, CAMERA_PARAMS['height']-1), (0,255,0), 2)

            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))

        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")

    # -----------------------
    # Diffdrive oscillation
    # -----------------------
    def timer_callback(self):
        if self.state != "TRACK_OBJECT":
            return
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        yaw_rad = math.radians(self.yaw_amplitude_deg) * math.sin(2*math.pi*self.yaw_frequency_hz*t)
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.angular.z = yaw_rad
        twist_msg.twist.linear.x = 0.0
        self.cmd_pub.publish(twist_msg)


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
