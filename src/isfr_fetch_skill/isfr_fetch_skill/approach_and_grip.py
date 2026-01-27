import math
import time
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
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from isfr_bot_msgs.action import SetGripperPosition, SetGripperOpening, ApproachAndGrabTask
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# --- Configuration ---
CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "center_u": 640//2,
    "center_v": 480//2,
    "FOV": 1.57
}
GRIPPER_HOME = (0.2, 0.18)
GRIPPER_OPEN = 0.02
# alignment
KP_YAW = 0.005  # Proportional yaw gain
MAX_YAW_VEL = 0.5 # yaw Rad/s limit
TOLERANCE_PX = 2 # yaw tolerance
KP_GRIPPER_Z = 0.002 # Sensitivity for vertical arm movement
ALIGN_IGNORE_DISTANCE = 0.25 # stop aligning when closer than this (risk of object exceeding camera FOV)
# approachment
CAMERA_APPROACH_DISTANCE = 0.12
APPROACH_DISTANCE_THRESH = 0.01
CAMERA_TO_GRIPPER_OFFSET = (-0.0617, 0.07)
MAX_FORWARD_VEL = 0.1 # m/s
KP_FORWARD = 0.5
GRASPING_OVERSHOOT_DEPTH = 0.03 # make sure the object rests in the center of the grippers
# retreat
LIFT_HEIGHT = 0.01  # 1cm
DRIVE_BACK_DISTANCE = 0.25

class ApproachGrip(Node):
    def __init__(self):
        super().__init__('approach_grip')
        self.bridge = CvBridge()
        
        # --- Helper Classes ---
        self.odom_tracker = OdomObjectTracker(CAMERA_PARAMS)
        self.visual_refiner = DepthTemplateRefiner(self, search_margin=0.25)

        # action server:
        self.action_group = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self,
            ApproachAndGrabTask,
            'approach_and_grab',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_group
        )

        # --- State Machine ---
        self.state = "IDLE"
        self.target_label = ""
        self.goal_handle = None
        self.start_time = None
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

        self.arm_is_moving = False
        self.current_gripper_z = GRIPPER_HOME[1]
        self.stop_tracking = False
        
    # =========================================
    # CALLBACKS
    # =========================================

    def goal_callback(self, goal_request):
        if self.state != "IDLE":
            self.get_logger().warn("Goal rejected: already busy")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def send_feedback(self, state_str):
        if self.goal_handle:
            feedback = ApproachAndGrabTask.Feedback()
            feedback.state = state_str
            feedback.stamp = self.get_clock().now().to_msg()
            self.goal_handle.publish_feedback(feedback)

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        self.goal_handle = goal_handle
        self.target_label = goal_handle.request.target_label
        self.start_time = self.get_clock().now()
        self.stop_tracking = False
        self.update_state("WAIT_FOR_OBJECTS")
        self.get_logger().info(f"ℹ️ Starting task for: {self.target_label}")

        loop_rate = self.create_rate(10)

        while rclpy.ok() and self.state != "COMPLETED":
            if self.goal_handle.is_cancel_requested:
                return ApproachAndGrabTask.Result(success=False, message="Task canceled")

            # Check for timeout
            if self.state == "WAIT_FOR_OBJECTS":
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                if elapsed > 15.0:
                     return ApproachAndGrabTask.Result(success=False, message="Timeout")

            loop_rate.sleep() # Tiny sleep to let other threads/callbacks work

        goal_handle.succeed()
        result = ApproachAndGrabTask.Result(success=True, message="Object successfully grabbed and retreated.")
        self.state = "IDLE"
        return result
    
    def update_state(self, new_state):
        self.state = new_state
        self.get_logger().info(f"Transitioning to state: {new_state}")
        self.send_feedback(new_state)

    def odom_callback(self, msg):
        if self.state == "IDLE": return
        q = msg.pose.pose.orientation
        t = msg.pose.pose.position
        T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = [t.x, t.y, t.z]
        self.current_odom_matrix = T

        if self.state == "RETREAT":
            curr_x = T[0, 3]
            curr_y = T[1, 3]
            dist_moved = math.sqrt((curr_x - self.retreat_start_odom_x)**2 + (curr_y - self.retreat_start_odom_y)**2)
            
            if dist_moved >= DRIVE_BACK_DISTANCE:
                self.get_logger().info("✅ 🙌 🥳 🎉 Object pickup has succeeded.")
                self.stop_robot()
                self.update_state("COMPLETED")
            else:
                self.drive_backward_step()

    def grasp_objects_callback(self, msg):
        if self.state == "IDLE": return
        # Only listen if we are waiting for an object
        if self.state != "WAIT_FOR_OBJECTS" or not msg.objects:
            return
        self.get_logger().info("ℹ️ Choosing target object")
        self.state_wait_for_object(msg)

    def depth_callback(self, msg):
        if self.state == "IDLE": return
        # get status
        if self.current_odom_matrix is None: return
        T_base_cam = self.get_camera_transform()
        if T_base_cam is None: return
        depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')

        # state machine
        if self.state == "LOCK_TARGET":
            self.state_lock_target(depth_image, T_base_cam)
        elif self.state in ["ALIGN_OBJECT", "APPROACH_OBJECT"]:
            self.tracking_states(depth_image, T_base_cam)
    
    # =========================================
    # STATES
    # =========================================   

    def state_wait_for_object(self, msg):
        # Pick object closest to image center
        objects_typed = [o for o in msg.objects if o.label == self.target_label]
        if len(objects_typed) == 0: return
        target_obj = min(objects_typed, key=lambda o: abs((o.xmin + o.xmax)/2 - CAMERA_PARAMS['width']/2))
        
        self.get_logger().info(f"Target Selected: {target_obj.label}. Transitioning to LOCK_TARGET.")
        
        # Store object data and transition to LOCK state
        # We do NOT track yet, we wait for the next Depth frame to get Z and lock the 3D point.
        self.pending_object_data = target_obj
        self.update_state("LOCK_TARGET")

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
            self.update_state("HOME_GRIPPER")
            gr_x, gr_z = GRIPPER_HOME
            self.send_gripper_pos_goal(gr_x, gr_z)
            self.send_gripper_opening_goal(GRIPPER_OPEN)
        else:
            self.get_logger().warn("Lock failed (bad depth or bad box). Retrying...")

    def tracking_states(self, depth_image, T_base_cam):
        # 1. Get Coarse Guess from Odom
        uv_guess = self.odom_tracker.get_projected_pixel(self.current_odom_matrix, T_base_cam)
        
        main_debug_img = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        main_debug_img = cv2.cvtColor(main_debug_img, cv2.COLOR_GRAY2BGR)

        angular_vel_z = 0.0
        linear_vel_x = 0.0

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

                if (self.state in ["ALIGN_OBJECT", "APPROACH_OBJECT"]) and (not self.stop_tracking):
                    angular_vel_z, target_z, move_arm = self.alignment_control(u_fine, v_fine, angular_vel_z)
                    if move_arm:
                        self.send_gripper_pos_goal(GRIPPER_HOME[0], target_z)
                if self.state == "APPROACH_OBJECT":
                    linear_vel_x = self.approach_object(depth_image, u_fine, v_fine)

                # 1. Draw only high-level overlays on the main feed
                cv2.rectangle(main_debug_img, (rx1, ry1), (rx2, ry2), (255, 255, 0), 2) # Cyan ROI
                cv2.circle(main_debug_img, (u_fine, v_fine), 5, (0, 0, 255), -1)        # Red Grab Point
                cv2.rectangle(main_debug_img, (bx, by), (bx+bw, by+bh), (0, 0, 255), 2) # Red Object Box
                cv2.rectangle(main_debug_img, (CAMERA_PARAMS['center_u'], 0), (CAMERA_PARAMS['center_u'], CAMERA_PARAMS['height']), (255, 0, 0), 1) # Center
                cv2.rectangle(main_debug_img, (0, CAMERA_PARAMS['center_v']), (CAMERA_PARAMS['width'], CAMERA_PARAMS['center_v']), (255, 0, 0), 1) # Center
                
            else:
                # Visual tracking lost (maybe occlusion?), fallback to just Green Cross
                cv2.putText(main_debug_img, "VISUAL LOST", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                if self.state == "APPROACH_OBJECT":
                    linear_vel_x = self.approach_object(depth_image, u_guess, v_guess)
        
        else:
            self.get_logger().warn("Odom target out of view")

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.angular.z = float(angular_vel_z)
        cmd.twist.linear.x = float(linear_vel_x if linear_vel_x is not None else 0.0)
        self.cmd_pub.publish(cmd)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(main_debug_img, 'bgr8'))

    def state_raise_and_reach_gripper(self):
        target_z = self.current_gripper_z + CAMERA_TO_GRIPPER_OFFSET[1]
        final_x = GRIPPER_HOME[0] + CAMERA_APPROACH_DISTANCE + CAMERA_TO_GRIPPER_OFFSET[0] + GRASPING_OVERSHOOT_DEPTH
        self.send_gripper_pos_goal(final_x, target_z)
        self.current_gripper_z = target_z

    def state_touch_that_thing(self):
        self.send_gripper_opening_goal(0.0)

    def state_deadlift(self):
        retract_x = GRIPPER_HOME[0]
        target_z = self.current_gripper_z + LIFT_HEIGHT
        self.send_gripper_pos_goal(retract_x, target_z)

    def state_prepare_retreat(self):
        if self.current_odom_matrix is not None:
            self.retreat_start_odom_x = self.current_odom_matrix[0, 3]
            self.retreat_start_odom_y = self.current_odom_matrix[1, 3]
            self.update_state("RETREAT")
            self.get_logger().info("ℹ️ Initialised retreat protocol. Retreating now...")
        else:
            self.get_logger().error("Odom lost! Cannot drive back safely.")

    # =========================================
    # HELPER
    # =========================================

    def approach_object(self, depth_image, u_fine, v_fine):
        current_depth = depth_image[v_fine, u_fine]
        if current_depth <= 0 or np.isnan(current_depth): return
        if current_depth <= ALIGN_IGNORE_DISTANCE: self.stop_tracking = True
        error_dist = current_depth - CAMERA_APPROACH_DISTANCE
        if error_dist <= APPROACH_DISTANCE_THRESH:
            self.get_logger().info("ℹ️ Approach distance reached. Raising gripper...")
            self.stop_robot()
            self.update_state("RAISE_AND_REACH_GRIPPER")
            self.state_raise_and_reach_gripper()
        else:
            return np.clip(error_dist * KP_FORWARD, 0.0, MAX_FORWARD_VEL)

    def alignment_control(self, u_fine, v_fine, angular_vel_z):
        target_z = -1
        error_u = u_fine - CAMERA_PARAMS['center_u']
        error_v = v_fine - CAMERA_PARAMS['center_v']
        u_centered = abs(error_u) <= TOLERANCE_PX
        v_centered = abs(error_v) <= TOLERANCE_PX
        if not u_centered:
            angular_vel_z = -float(error_u) * KP_YAW
            angular_vel_z = np.clip(angular_vel_z, -MAX_YAW_VEL, MAX_YAW_VEL)
        update_target_z = (not v_centered) and (not self.arm_is_moving)
        if update_target_z:
            target_z = self.current_gripper_z - (error_v * KP_GRIPPER_Z)
            self.get_logger().info(f"ℹ️ Moving arm z from {self.current_gripper_z} to {target_z}")
            self.current_gripper_z = target_z
        if u_centered and v_centered and (not self.state == "APPROACH_OBJECT"):
            self.get_logger().info(f"ℹ️ Robot orientation is centered on object. Initiating approach")
            self.update_state("APPROACH_OBJECT")
        return angular_vel_z, target_z, update_target_z

    def send_gripper_opening_goal(self, opening):
        if not self.gripper_opening_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server /set_gripper_opening not available!")
            return
        goal_msg = SetGripperOpening.Goal()
        goal_msg.opening = float(opening)
        send_goal_future = self.gripper_opening_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_opening_response_callback)

    def send_gripper_pos_goal(self, x, z):
        # Wait for server to be available
        if not self.gripper_pos_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server /set_gripper_position not available!")
            return

        goal_msg = SetGripperPosition.Goal()
        goal_msg.x = float(x)
        goal_msg.z = float(z)
        send_goal_future = self.gripper_pos_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_pos_response_callback)
        self.arm_is_moving = True

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
        self.arm_is_moving = False
        if self.state == "HOME_GRIPPER":
            self.get_logger().info(f"ℹ️ Gripper is home. Starting object tracking...")
            self.update_state("ALIGN_OBJECT")
        elif self.state == "RAISE_AND_REACH_GRIPPER":
            self.get_logger().info(f"ℹ️ Gripper raised. I can going to touch you now 💀...")
            self.update_state("TOUCH_THAT_THING")
            self.state_touch_that_thing()
        elif self.state == "LIFT_AND_RETRACT":
            self.get_logger().info(f"ℹ️ Object is lifted. Retreating away from the crime scene..")
            self.update_state("PREPARE_RETREAT")
            self.state_prepare_retreat()

    def gripper_opening_callback(self, _):
        if self.state in ["HOME_GRIPPER", "TRACK_OBJECT"]:
            return
        if self.state == "TOUCH_THAT_THING":
            self.get_logger().info(f"ℹ️ Dat ding is helemaal betast. 💪Nu nog even deadliften..")
            self.update_state("LIFT_AND_RETRACT")
            self.state_deadlift()

    def stop_robot(self):
        self.get_logger().info(f"🛑 Stopping locomotion")
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)
            

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
        
    def drive_backward_step(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(-MAX_FORWARD_VEL) # Negative for reverse
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ApproachGrip()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        # rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()