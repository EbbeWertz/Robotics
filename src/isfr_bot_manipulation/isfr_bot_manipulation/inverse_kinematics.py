#!/usr/bin/env python3
from control_msgs.action import GripperCommand
import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from isfr_bot_msgs.action import SetGripperOpening, SetGripperPosition
import numpy as np
import math

import tf2_ros
from geometry_msgs.msg import TransformStamped

INTERPOLATION_POINTS_PER_METER = 100 # interpolation step = 1cm
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

    # convert any angle to range (-180° to +180°)
def  wrap_to_pi(t):
    return (t+math.pi) % (2*math.pi)-math.pi


class OpenManipulatorIKNode(Node):

    def __init__(self):
        super().__init__('openmanipulator_ik_node')

        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.left_gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_left_controller/gripper_cmd'
        )
        
        self.right_gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_right_controller/gripper_cmd'
        )


        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._kinematics_init()

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
    
    def _extract_constant_kinematics(self):
        # relative coordinates
        x_es, _, z_es = self._get_translation('arm_shoulder_link', 'arm_elbow_link')
        x_we, _, z_we = self._get_translation('arm_elbow_link', 'arm_wrist_link')
        x_gw, _, z_gw = self._get_translation('arm_wrist_link', 'gripper_center_link')
        # arm lengths
        self.L_se = math.hypot(x_es, z_es)   # shoulder → elbow
        self.L_ew = math.hypot(x_we, z_we)   # elbow → wrist
        self.L_wg = math.hypot(x_gw, z_gw)   # wrist → gripper center
        
    def _create_kinematic_offsets(self):
        x_sb, _, z_sb = self._get_translation('arm_base_link', 'arm_shoulder_link')
        x_es, _, z_es = self._get_translation('arm_shoulder_link', 'arm_elbow_link')
        thetas_abs = math.atan2(z_es, x_es)
        self.thetas_abs = thetas_abs
        
        # gripper xy from arm_base as origin to arm_shoulder as origin (shoulder  = kinematic zero)
        self.offset_gripperxy_to_kinematic = lambda x_gb, y_gb: (x_gb - x_sb, y_gb - z_sb)
        # kinematic angle to motor angle
        self.phi_s_to_motor_angle = lambda phi_s: wrap_to_pi(thetas_abs - phi_s)
        self.phi_e_to_motor_angle = lambda phi_e: wrap_to_pi(math.pi - self.thetas_abs - phi_e)
        self.phi_w_to_motor_angle = lambda phi_w: wrap_to_pi(math.pi - phi_w)

    # x,y = gripper relative to shoulder, alpha = absolute gripper orientation in radians
    def _solve_kinematics(self, x_gs, z_gs, alpha):
        # wrist coordinates (relative to shoulder):
        x_ws = x_gs - self.L_wg * math.cos(alpha)
        z_ws = z_gs - self.L_wg * math.sin(alpha)
        d = math.hypot(x_ws, z_ws) # distance wrist to shoulder
        # square helpers:
        L_se_sq = self.L_se * self.L_se
        L_ew_sq = self.L_ew * self.L_ew
        d_sq = d * d
        # absolute / helper angles
        theta_d = math.atan2(z_ws, x_ws)
        theta_sw = math.acos((L_ew_sq-L_se_sq-d_sq)/(-2*d*self.L_se))
        # kinematic angles
        phi_s = theta_d + theta_sw
        phi_e = math.acos((d_sq - L_ew_sq - L_se_sq)/(-2*self.L_ew*self.L_se))
        phi_w = (math.pi - (phi_s + phi_e - math.pi)) + alpha
        return phi_s, phi_e, phi_w

    def _kinematics_init(self):
        self.get_logger().info("Waiting for TF to extract arm geometry...")
        while rclpy.ok():
            try:
                self._extract_constant_kinematics()
                self._create_kinematic_offsets()
                x_gb, _, z_gb = self._get_translation('arm_base_link', 'gripper_center_link')
                x_gs, _, z_gs = self._get_translation('arm_shoulder_link', 'gripper_center_link')
                self.get_logger().info(
                    "=== Extracted arm kinematics (motor=0 reference) ===\n"
                    f"Segment lengths:\n"
                    f"  L_se shoulder→elbow : {self.L_se:.4f} m\n"
                    f"  L_ew elbow→wrist    : {self.L_ew:.4f} m\n"
                    f"  L_wg wrist→gripper  : {self.L_wg:.4f} m\n\n"
                    f"Shoulder angle: {math.degrees(self.thetas_abs):.2f} deg\n"
                    f"Current gripper pos:\n"
                    f"  x : {x_gb:.4f} m (kinematic x = {x_gs:.4f} m)\n"
                    f"  z : {z_gb:.4f} m (kinematic x = {z_gs:.4f} m)\n"
                    "===================================================="
                )
                return
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("TF not available for kinematics extraction")

    def _check_safe_target(self, x_gs, z_gs):
        MIN_RADIUS = 0.12  # 12 cm
        MAX_REACH = self.L_se + self.L_ew + self.L_wg- 1e-3

        r = math.hypot(x_gs, z_gs)
        if r < MIN_RADIUS:
            self.get_logger().error("Target too close to shoulder (must be >= 0.05 m)")
            return False, "Target too close to shoulder (must be >= 0.05 m)"
        
        if r > MAX_REACH:
            self.get_logger().error("Target out of reach (arm lengts literally can't reach that far)")
            return False, "Target out of reach (arm lengts literally can't reach that far)"
        
        if x_gs < 0 or z_gs < 0:
            self.get_logger().error("Negative workspace not allowed (x and z must be positive)")
            return False, "Negative workspace not allowed (x and z must be positive)"
        
        return True, ''


    # # --------------------------------------------------
    # COMMAND PUBLISHERS
    # --------------------------------------------------
    def publish_arm(self, anglesTrajectory, time):
        traj = JointTrajectory()
        traj.joint_names = [
            'arm_orientation_motor',
            'arm_shoulder_motor',
            'arm_elbow_motor',
            'arm_wrist_motor'
        ]
        
        for i, angles in enumerate(anglesTrajectory):
            n = len(anglesTrajectory)
            t = time * i / (n - 1 if n > 1 else 1)
            sec = int(t)
            nanosec = int((t - sec) * 1e9)

            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec
            traj.points.append(point)
        self.arm_pub.publish(traj)

    def publish_gripper(self, opening):
        goal = GripperCommand.Goal()
        goal.command.position = clamp(opening, JOINT_LIMITS['gripper_left_joint_motor'])
        self.left_gripper_client.wait_for_server()
        self.right_gripper_client.wait_for_server()

        self.left_gripper_client.send_goal_async(goal)
        self.right_gripper_client.send_goal_async(goal)

    # --------------------------------------------------
    # ACTION CALLBACKS
    # --------------------------------------------------
    
    def execute_gripper_opening(self, goal_handle):
        opening = goal_handle.request.opening
        limit_low, limit_high = JOINT_LIMITS['gripper_left_joint_motor']
        if opening < limit_low or opening > limit_high:
            self.get_logger().error(f"Gripper opening should be in the range {limit_low}->{limit_high}")
            goal_handle.abort()
            return SetGripperOpening.Result(success=False, message=f"Gripper opening should be in the range {limit_low}->{limit_high}")

        self.publish_gripper(opening)
        goal_handle.succeed()
        return SetGripperOpening.Result(success=True)

    def execute_gripper_position(self, goal_handle):
        new_x = goal_handle.request.x
        new_z = goal_handle.request.z
        old_x, _, old_z = self._get_translation('arm_base_link', 'gripper_center_link')

        new_x_gs, new_z_gs = self.offset_gripperxy_to_kinematic(new_x, new_z)
        old_x_gs, old_z_gs = self.offset_gripperxy_to_kinematic(old_x, old_z)

        safe, res_msg = self._check_safe_target(new_x_gs, new_z_gs)
        if not safe:
            goal_handle.abort()
            return SetGripperPosition.Result(success=False, message=res_msg)

            
        distance = math.hypot(new_x_gs - old_x_gs, new_z_gs - old_z_gs)
        num_of_points = int(distance * INTERPOLATION_POINTS_PER_METER)+1
        x_points = np.linspace(old_x_gs, new_x_gs, num_of_points+1, endpoint=True)[1:]
        z_points = np.linspace(old_z_gs, new_z_gs, num_of_points+1, endpoint=True)[1:]

        self.get_logger().info(
            "=====================================\n"
            "Moving Gripper:\n"
            f"  x : from {old_x:.4f} m to {new_x:.4f} m\n"
            f"  z : from {old_z:.4f} m to {new_z:.4f} m\n"
            f"Total distance: {distance:.4f} m\n"
            f"Num of steps: {num_of_points}\n"
            "====================================="
        )

        phi_s, phi_e, phi_w = None, None, None
        motor_s, motor_e, motor_w = None, None, None

        motor_angles = []

        for i in range(num_of_points):
            phi_s, phi_e, phi_w = self._solve_kinematics(x_points[i], z_points[i], 0)
            motor_s = self.phi_s_to_motor_angle(phi_s)
            motor_e = self.phi_e_to_motor_angle(phi_e)
            motor_w = self.phi_w_to_motor_angle(phi_w)
            motor_angles.append([0.0, motor_s, motor_e, motor_w])

        self.publish_arm(motor_angles, 1)
        self.get_logger().info(
            "=====================================\n"
            "--> final arm angles...\n"
            f"shoulder: kinematic = {math.degrees(phi_s):.2f} deg, motor = {math.degrees(motor_s):.2f} deg\n"
            f"elbow: kinematic = {math.degrees(phi_e):.2f} deg, motor = {math.degrees(motor_e):.2f} deg\n"
            f"wrist: kinematic = {math.degrees(phi_w):.2f} deg, motor = {math.degrees(motor_w):.2f} deg\n"
            "====================================="
        )
        goal_handle.succeed()
        return SetGripperPosition.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = OpenManipulatorIKNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
