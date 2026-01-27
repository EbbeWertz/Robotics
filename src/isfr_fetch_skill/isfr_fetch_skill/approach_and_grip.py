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
from .DepthTemplateMatcherTracker import DepthTemplateRefiner
from rclpy.action import ActionClient
from isfr_bot_msgs.action import SetGripperPosition, SetGripperOpening

# --- Configuration ---
CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "center_u": 640//2,
    "FOV": 1.57
}
GRIPPER_HOME = (0.25, 0.18)
GRIPPER_OPEN = 0.02
KP_YAW = 0.005  # Proportional yaw gain
MAX_YAW_VEL = 0.5 # yaw Rad/s limit
YAW_TOLERANCE_PX = 2 # yaw tolerance

TARGET_LABEL = "bottle"

class ApproachGrip(Node):
    def __init__(self):
        super().__init__('approach_grip')
        self.bridge = CvBridge()
        
        # --- Helper Classes ---
        self.odom_tracker = OdomObjectTracker(CAMERA_PARAMS)
        self.visual_refiner = DepthTemplateRefiner(self, search_margin=0.25)

        # --- State Machine ---
        # States: "WAIT_FOR_OBJECTS" -> "LOCK_TARGET" -> "HOME_GRIPPER" -> "TRACK_OBJECT"
        self.state = "WAIT_FOR_OBJECTS"
        self.pending_object_data = None 
        self.current_odom_matrix = None
        self.get_logger().info("ℹ️ Waiting for object candidates to be published")
        
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
        self.gripper_pos_client = ActionClient(self, SetGripperPosition, '/set_gripper_position')
        self.gripper_opening_client = ActionClient(self, SetGripperOpening, '/set_gripper_opening')
        
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
        self.get_logger().info("ℹ️ Choosing target object")
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
    
    # =========================================
    # STATES
    # =========================================   

    def state_wait_for_object(self, msg):
        # Pick object closest to image center
        objects_typed = [o for o in msg.objects if o.label == TARGET_LABEL]
        target_obj = min(objects_typed, key=lambda o: abs((o.xmin + o.xmax)/2 - CAMERA_PARAMS['width']/2))
        
        self.get_logger().info(f"Target Selected: {target_obj.label}. Transitioning to LOCK_TARGET.")
        
        # Store object data and transition to LOCK state
        # We do NOT track yet, we wait for the next Depth frame to get Z and lock the 3D point.
        self.pending_object_data = target_obj
        self.state = "LOCK_TARGET"

    def state_lock_target(self, depth_image, T_base_cam):
        self.get_logger().info("ℹ️ Locking target object location")
        obj = self.pending_object_data
        
        # 1. Calc Coarse Center (Grasp Point)
        bb_w = obj.xmax - obj.xmin
        bb_h = obj.ymax - obj.ymin
        u_norm = obj.graspline_u + (obj.graspline_width / 2.0)
        
        u = int(obj.xmin + bb_w * u_norm)
        v = int(obj.ymin + bb_h * obj.graspline_v)
        
        # Safe clamp
        u = np.clip(u, 0, CAMERA_PARAMS['width'] - 1)
        v = np.clip(v, 0, CAMERA_PARAMS['height'] - 1)
        
        z = depth_image[v, u]
        
        # 2. Init Odom Tracker (The Coarse Guesser)
        success_odom = self.odom_tracker.lock_target(u, v, z, self.current_odom_matrix, T_base_cam)
        
        # 3. Init Visual Refiner (The Fine Tracker)
        # We pass the full bounding box to capture the object shape
        success_vis = self.visual_refiner.initialize(
            depth_image, obj.xmin, obj.ymin, bb_w, bb_h, u, v, z
        )
        
        if success_odom and success_vis:
            self.get_logger().info(f"ℹ️ Locked target {obj.label} at [{u:.2f}, {v:.2f}]px, {z:.2f}m. Setting gripper to home...")
            self.state = "HOME_GRIPPER"
            gr_x, gr_z = GRIPPER_HOME
            self.send_gripper_pos_goal(gr_x, gr_z)
            self.send_gripper_opening_goal(GRIPPER_OPEN)
        else:
            self.get_logger().warn("Lock failed (bad depth or bad box). Retrying...")

    def state_track_object(self, depth_image, T_base_cam):
        # 1. Get Coarse Guess from Odom
        uv_guess = self.odom_tracker.get_projected_pixel(self.current_odom_matrix, T_base_cam)
        
        main_debug_img = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        main_debug_img = cv2.cvtColor(main_debug_img, cv2.COLOR_GRAY2BGR)

        angular_vel_z = 0.0

        if uv_guess:
            u_guess, v_guess = uv_guess
            
            # Draw Coarse Guess and search window (Cyan Cross)
            cv2.drawMarker(main_debug_img, (u_guess, v_guess), (255, 255, 0), cv2.MARKER_CROSS, 20, 2)
            
            z_approx = depth_image[v_guess, u_guess]
            if z_approx <= 0 or np.isnan(z_approx): z_approx = 1.0 # Default fallback
            
            result = self.visual_refiner.track(depth_image, u_guess, v_guess, z_approx)
            
            if result:
                u_fine, v_fine, (bx, by, bw, bh), refine_info = result
                rx1, ry1, rx2, ry2 = refine_info["roi_rect"]

                # control logic: rotate around z axis to align u_fine to center
                error_u = u_fine - CAMERA_PARAMS['center_u']
                is_centered = abs(error_u) <= YAW_TOLERANCE_PX
                if not is_centered:
                    angular_vel_z = -float(error_u) * KP_YAW
                    angular_vel_z = np.clip(angular_vel_z, -MAX_YAW_VEL, MAX_YAW_VEL)
                else:
                    self.get_logger().info(f"ℹ️ Robot orientation is centered on object")

                # 1. Draw only high-level overlays on the main feed
                cv2.rectangle(main_debug_img, (rx1, ry1), (rx2, ry2), (255, 255, 0), 2) # Cyan ROI
                cv2.circle(main_debug_img, (u_fine, v_fine), 5, (0, 0, 255), -1)        # Red Grab Point
                cv2.rectangle(main_debug_img, (bx, by), (bx+bw, by+bh), (0, 0, 255), 2) # Red Object Box
                cv2.rectangle(main_debug_img, (CAMERA_PARAMS['center_u'], 0), (CAMERA_PARAMS['center_u'], CAMERA_PARAMS['height']), (0, 255, 255), 2) # Center
                
            else:
                # Visual tracking lost (maybe occlusion?), fallback to just Green Cross
                cv2.putText(main_debug_img, "VISUAL LOST", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        
        else:
            self.get_logger().warn("Odom target out of view")

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.angular.z = angular_vel_z
        self.cmd_pub.publish(cmd)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(main_debug_img, 'bgr8'))

    # =========================================
    # HELPER
    # =========================================

    def send_gripper_opening_goal(self, opening):
        if not self.gripper_opening_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server /set_gripper_opening not available!")
            return
        goal_msg = SetGripperOpening.Goal()
        goal_msg.opening = opening
        send_goal_future = self.gripper_opening_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_opening_response_callback)

    def send_gripper_pos_goal(self, x, z):
        # Wait for server to be available
        if not self.gripper_pos_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server /set_gripper_position not available!")
            return

        goal_msg = SetGripperPosition.Goal()
        goal_msg.x = x
        goal_msg.z = z
        send_goal_future = self.gripper_pos_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_pos_response_callback)

    def goal_pos_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return
        goal_handle.get_result_async().add_done_callback(self.gripper_pos_callback)

    def goal_opening_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return
        goal_handle.get_result_async().add_done_callback(self.gripper_opening_callback)

    def gripper_pos_callback(self, _):
        if self.state == "HOME_GRIPPER":
            self.get_logger().info(f"ℹ️ Gripper is home. Starting object tracking...")
            self.state = "TRACK_OBJECT"

    def gripper_opening_callback(self, _):
        if self.state in ["HOME_GRIPPER", "TRACK_OBJECT"]:
            return

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