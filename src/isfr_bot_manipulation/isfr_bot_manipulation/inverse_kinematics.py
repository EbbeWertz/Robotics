#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from isfr_bot_msgs.action import SetGripperOpening, SetGripperPosition
import math

import tf2_ros
from geometry_msgs.msg import TransformStamped

# Joint limits
JOINT_LIMITS = {
    "arm_orientation_motor": (-math.pi, math.pi),
    "arm_shoulder_motor": (-1.5, 1.5),
    "arm_elbow_motor": (-1.5, 1.4),
    "arm_wrist_motor": (-1.7, 1.97),
    "gripper_left_joint_motor": (-0.011, 0.02),
    "gripper_right_joint_motor": (-0.011, 0.02)
}


def clamp(value, limits):
    return max(min(value, limits[1]), limits[0])


class OpenManipulatorIKNode(Node):

    def __init__(self):
        super().__init__('openmanipulator_ik_node')

        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.gripper_left_pub = self.create_publisher(
            JointTrajectory, '/gripper_left_controller/joint_trajectory', 10)

        self.gripper_right_pub = self.create_publisher(
            JointTrajectory, '/gripper_right_controller/joint_trajectory', 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Extract initial kinematics from TF
        self._extract_constant_kinematics()
        self._create_kinematic_offsets()


        # Action servers
        self._gripper_action_server = ActionServer(
            self, SetGripperOpening, 'set_gripper_opening',
            self.execute_gripper_opening)

        self._position_action_server = ActionServer(
            self, SetGripperPosition, 'set_gripper_position',
            self.execute_gripper_position)

    # --------------------------------------------------
    # TF-BASED KINEMATICS EXTRACTION
    # --------------------------------------------------
    def _get_translation(self, parent, child) -> tuple:
        tf: TransformStamped = self.tf_buffer.lookup_transform(
            parent, child, rclpy.time.Time())
        t = tf.transform.translation
        return (t.x, t.y, t.z)
    
    # convert any angle to range (-180° to +180°)
    def  _wrap_to_pi(t):
        return (t+math.pi) % (2*math.pi)-math.pi
    
    def _extract_constant_kinematics(self):
        # relative coordinates
        x_es, _, z_es = self._get_translation('arm_shoulder_link', 'arm_elbow_link')
        x_we, _, z_we = self._get_translation('arm_elbow_link', 'arm_wrist_link')
        x_gw, _, z_gw = self._get_translation('arm_wrist_link', 'gripper_center_link')
        x_ws, _, z_ws = self._get_translation('arm_shoulder_link', 'arm_wrist_link')
        # arm lengths
        self.L_se = math.hypot(x_es, z_es)   # shoulder → elbow
        self.L_ew = math.hypot(x_we, z_we)   # elbow → wrist
        self.L_wg = math.hypot(x_gw, z_gw)   # wrist → gripper center
        self.d = math.hypot(x_ws, z_ws) # distance wrist to shoulder

    def _create_kinematic_offsets(self):
        x_sb, _, z_sb = self._get_translation('arm_shoulder_link', 'arm_base_link')
        x_es, _, z_es = self._get_translation('arm_shoulder_link', 'arm_elbow_link')
        thetas_abs = math.atan2(z_es, x_es)
        
        # gripper xy from arm_base as origin to arm_shoulder as origin (shoulder  = kinematic zero)
        self.offset_gripperxy_to_kinematic = lambda x_gb, y_gb: (x_gb - x_sb, y_gb - z_sb)
        # kinematic angle to motor angle
        self.phi_s_to_motor_angle = lambda phi_s: self._wrap_to_pi(thetas_abs - phi_s)
        self.phi_e_to_motor_angle = lambda phi_e: self._wrap_to_pi(math.pi - thetas_abs - phi_e)
        self.phi_w_to_motor_angle = lambda phi_w: self._wrap_to_pi(math.pi - phi_w)

    # x,y = gripper relative to shoulder, alpha = absolute gripper orientation in radians
    def _solve_kinematics(self, x_gs, z_gs, alpha):
        # wrist coordinates (relative to shoulder):
        x_ws = x_gs - self.L_wg * math.cos(alpha)
        z_ws = z_gs - self.L_wg * math.sin(alpha)
        # square helpers:
        L_se_sq = self.L_se * self.L_se
        L_ew_sq = self.L_ew * self.L_ew
        d_sq = self.d * self.d
        # absolute / helper angles
        theta_d = math.atan2(z_ws, x_ws)
        theta_sw = math.acos((L_ew_sq-L_se_sq-d_sq)/(-2*self.d*self.L_se))
        # kinematic angles
        phi_s = theta_d + theta_sw
        phi_e = math.acos((d_sq - L_ew_sq - L_se_sq)/(-2*self.L_ew*self.L_se))
        phi_w = (math.pi - (phi_s + phi_e - math.pi)) + alpha
        return phi_s, phi_e, phi_w


    # def _extract_kinematics(self):
    #     self.get_logger().info("Waiting for TF to extract arm geometry...")
    #     while rclpy.ok():
    #         try:
    #             # Translations from parent → child
    #             sx, _, sz = self._get_translation('arm_base_link', 'arm_shoulder_link')
    #             ex, _, ez = self._get_translation('arm_shoulder_link', 'arm_elbow_link')
    #             wx, _, wz = self._get_translation('arm_elbow_link', 'arm_wrist_link')
    #             gx, _, gz = self._get_translation('arm_wrist_link', 'gripper_center_link')

    #             # Shoulder position relative to arm_base_link
    #             self.xs = sx
    #             self.zs = sz

    #             # Segment lengths (2D in X-Z plane)
    #             self.L1 = math.hypot(ex, ez)   # shoulder → elbow
    #             self.L2 = math.hypot(wx, wz)   # elbow → wrist
    #             self.Lw = math.hypot(gx, gz)   # wrist → gripper center

    #             # Absolute orientations
    #             theta1_abs = math.atan2(ez, ex)
    #             theta2_abs = math.atan2(wz, wx)
    #             theta3_abs = math.atan2(gz, gx)

    #             # Kinematic zero angles (motor=0)
    #             self.theta1_0 = theta1_abs
    #             self.theta2_0 = theta2_abs - theta1_abs
    #             self.theta3_0 = theta3_abs - theta2_abs

    #             self.get_logger().info(
    #                 "=== Extracted arm kinematics (motor=0 reference) ===\n"
    #                 f"Segment lengths:\n"
    #                 f"  L1 shoulder→elbow : {self.L1:.4f} m\n"
    #                 f"  L2 elbow→wrist    : {self.L2:.4f} m\n"
    #                 f"  Lw wrist→gripper  : {self.Lw:.4f} m\n\n"
    #                 f"Absolute segment orientations (base frame):\n"
    #                 f"  shoulder→elbow θ1_abs : {math.degrees(theta1_abs):.2f} deg\n"
    #                 f"  elbow→wrist    θ2_abs : {math.degrees(theta2_abs):.2f} deg\n"
    #                 f"  wrist→gripper  θ3_abs : {math.degrees(theta3_abs):.2f} deg\n\n"
    #                 f"Kinematic joint angles at motor=0:\n"
    #                 f"  joint1 θ1_0 : {math.degrees(self.theta1_0):.2f} deg\n"
    #                 f"  joint2 θ2_0 : {math.degrees(self.theta2_0):.2f} deg\n"
    #                 f"  joint3 θ3_0 : {math.degrees(self.theta3_0):.2f} deg\n"
    #                 "===================================================="
    #             )
    #             return

    #         except Exception:
    #             rclpy.spin_once(self, timeout_sec=0.1)

    #     raise RuntimeError("TF not available for kinematics extraction")

    # # --------------------------------------------------
    # # UPDATED PLANAR IK SOLVER
    # # --------------------------------------------------

    # def solve_ik(self, x, z):
    #     """
    #     Planar IK for the OpenManipulatorX in the X-Z plane.
    #     Targeting the center of the gripper with a horizontal constraint.
    #     """
    #     # 1. Wrist Center (WC) Calculation
    #     # To keep gripper horizontal, the wrist must be Lw behind the target x
    #     x_wc = x - self.Lw
    #     z_wc = z
        
    #     # 2. Vector from Shoulder to Wrist Center
    #     dx = x_wc - self.xs
    #     dz = z_wc - self.zs
    #     r2 = dx**2 + dz**2
    #     r = math.sqrt(r2)

    #     # 3. Check reachability
    #     if r > (self.L1 + self.L2) or r < abs(self.L1 - self.L2):
    #         self.get_logger().error(f"Target ({x}, {z}) unreachable. Dist: {r:.4f}")
    #         raise ValueError("Target unreachable")

    #     # 4. Elbow Angle (Law of Cosines)
    #     # Angle alpha is the interior angle between L1 and L2
    #     cos_alpha = (self.L1**2 + self.L2**2 - r2) / (2 * self.L1 * self.L2)
    #     cos_alpha = max(-1.0, min(1.0, cos_alpha)) # Clamp for safety
    #     alpha = math.acos(cos_alpha)
        
    #     # The kinematic elbow angle (t2) is typically 180 - alpha
    #     # We use the 'elbow-up' configuration
    #     t2 = math.pi - alpha

    #     # 5. Shoulder Angle (t1)
    #     # Angle from horizontal to WC + angle between L1 and radius r
    #     phi = math.atan2(dz, dx)
    #     cos_beta = (self.L1**2 + r2 - self.L2**2) / (2 * self.L1 * r)
    #     cos_beta = max(-1.0, min(1.0, cos_beta))
    #     beta = math.acos(cos_beta)
        
    #     t1 = phi + beta # Elbow-up

    #     # 6. Wrist Angle (t3)
    #     # To keep gripper horizontal: t1 + t2 + t3 = 0 (relative to horizon)
    #     t3 = -(t1 + t2)

    #     # 7. Convert to motor commands
    #     # Subtract the 'motor=0' offsets found during TF extraction
    #     t1_motor = t1 - self.theta1_0
    #     t2_motor = t2 - self.theta2_0
    #     t3_motor = t3 - self.theta3_0

    #     # 8. Clamp to joint limits
    #     t1_motor_clamped = clamp(t1_motor, JOINT_LIMITS["arm_shoulder_motor"])
    #     t2_motor_clamped = clamp(t2_motor, JOINT_LIMITS["arm_elbow_motor"])
    #     t3_motor_clamped = clamp(t3_motor, JOINT_LIMITS["arm_wrist_motor"])

    #     return t1_motor_clamped, t2_motor_clamped, t3_motor_clamped

    # # --------------------------------------------------
    # COMMAND PUBLISHERS
    # --------------------------------------------------
    def publish_arm(self, yaw, shoulder, elbow, wrist):
        traj = JointTrajectory()
        traj.joint_names = [
            'arm_orientation_motor',
            'arm_shoulder_motor',
            'arm_elbow_motor',
            'arm_wrist_motor'
        ]
        point = JointTrajectoryPoint()
        point.positions = [yaw, shoulder, elbow, wrist]
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.arm_pub.publish(traj)

    def publish_gripper(self, opening):
        for joint, pub in [
            ('gripper_left_joint_motor', self.gripper_left_pub),
            ('gripper_right_joint_motor', self.gripper_right_pub)
        ]:
            traj = JointTrajectory()
            traj.joint_names = [joint]
            point = JointTrajectoryPoint()
            point.positions = [clamp(opening, JOINT_LIMITS[joint])]
            point.time_from_start.sec = 1
            traj.points.append(point)
            pub.publish(traj)

    # --------------------------------------------------
    # ACTION CALLBACKS
    # --------------------------------------------------
    def execute_gripper_opening(self, goal_handle):
        self.publish_gripper(goal_handle.request.opening)
        goal_handle.succeed()
        return SetGripperOpening.Result(success=True)

    def execute_gripper_position(self, goal_handle):
        x = goal_handle.request.x
        z = goal_handle.request.z

        x_gs, z_gs = self.offset_gripperxy_to_kinematic(x,z)
        phi_s, phi_e, phi_w = self._solve_kinematics(x_gs, z_gs, 0)

        motor_s = self.phi_s_to_motor_angle(phi_s)
        motor_e = self.phi_e_to_motor_angle(phi_e)
        motor_w = self.phi_w_to_motor_angle(phi_w)

        self.get_logger().info(
            "Moving arm angles:\n"
            f"shoulder = {math.degrees(phi_s):.2f} (motor angle = {math.degrees(motor_s):.2f})\n"
            f"elbow = {math.degrees(phi_e):.2f} (motor angle = {math.degrees(motor_e):.2f})\n"
            f"wrist = {math.degrees(phi_w):.2f} (motor angle = {math.degrees(motor_w):.2f})\n"
        )

        self.publish_arm(0, motor_s, motor_e, motor_w)
        goal_handle.succeed()
        return SetGripperPosition.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = OpenManipulatorIKNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
