#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from isfr_bot_msgs.action import SetGripperOpening, SetGripperPosition
import math

# Arm segment lengths (meters)
L1 = 0.128  # shoulder → elbow
L2 = 0.124  # elbow → wrist
Lw = 0.0817  # wrist → gripper center

# Shoulder location relative to arm_base_link
xs = 0.012
zs = 0.0595

# Joint limits (from your PROTO)
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

def solve_ik(x, z):
    """
    Inverse kinematics for planar 2-link manipulator in x-z plane.
    Elbow-up configuration. Gripper aligned along +x.
    x,z: target gripper center position relative to arm_base_link
    """
    dx = x - xs - Lw
    dz = z - zs
    r2 = dx*dx + dz*dz
    cos_t2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2)

    if abs(cos_t2) > 1.0:
        raise ValueError("Target unreachable")
    
    t2 = math.acos(cos_t2)  # elbow-up
    phi = math.atan2(dz, dx)
    psi = math.atan2(L2 * math.sin(t2), L1 + L2 * math.cos(t2))
    t1 = phi - psi
    t3 = -(t1 + t2)  # wrist aligns gripper along +x

    # Clamp to limits
    t1 = clamp(t1, JOINT_LIMITS["arm_shoulder_motor"])
    t2 = clamp(t2, JOINT_LIMITS["arm_elbow_motor"])
    t3 = clamp(t3, JOINT_LIMITS["arm_wrist_motor"])

    return t1, t2, t3

class OpenManipulatorIKNode(Node):
    def __init__(self):
        super().__init__('openmanipulator_ik_node')

        # Publishers
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_left_pub = self.create_publisher(JointTrajectory, '/gripper_left_controller/joint_trajectory', 10)
        self.gripper_right_pub = self.create_publisher(JointTrajectory, '/gripper_right_controller/joint_trajectory', 10)

        # Action servers
        self._gripper_action_server = ActionServer(
            self, SetGripperOpening, 'set_gripper_opening', self.execute_gripper_opening)
        self._position_action_server = ActionServer(
            self, SetGripperPosition, 'set_gripper_position', self.execute_gripper_position)

    # --------------------
    # ARM & GRIPPER PUBLISHERS
    # --------------------
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
        # Left finger
        traj_left = JointTrajectory()
        traj_left.joint_names = ['gripper_left_joint_motor']
        point_left = JointTrajectoryPoint()
        point_left.positions = [clamp(opening, JOINT_LIMITS["gripper_left_joint_motor"])]
        point_left.time_from_start.sec = 1
        traj_left.points.append(point_left)
        self.gripper_left_pub.publish(traj_left)

        # Right finger
        traj_right = JointTrajectory()
        traj_right.joint_names = ['gripper_right_joint_motor']
        point_right = JointTrajectoryPoint()
        point_right.positions = [clamp(opening, JOINT_LIMITS["gripper_right_joint_motor"])]
        point_right.time_from_start.sec = 1
        traj_right.points.append(point_right)
        self.gripper_right_pub.publish(traj_right)

    # --------------------
    # ACTION SERVER CALLBACKS
    # --------------------
    def execute_gripper_opening(self, goal_handle):
        opening = goal_handle.request.opening
        try:
            self.publish_gripper(opening)

            # Publish feedback
            feedback = SetGripperOpening.Feedback()
            feedback.current_opening = opening
            goal_handle.publish_feedback(feedback)

            # Send result
            result = SetGripperOpening.Result()
            result.success = True
            result.message = f"Gripper set to {opening:.3f} m"
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"Gripper command error: {e}")
            result = SetGripperOpening.Result()
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

    def execute_gripper_position(self, goal_handle):
        x = goal_handle.request.x
        z = goal_handle.request.z
        try:
            shoulder, elbow, wrist = solve_ik(x, z)
            yaw = 0.0  # can be parameterized separately

            # Send command to arm
            self.publish_arm(yaw, shoulder, elbow, wrist)

            # Publish feedback
            feedback = SetGripperPosition.Feedback()
            feedback.shoulder = shoulder
            feedback.elbow = elbow
            feedback.wrist = wrist
            goal_handle.publish_feedback(feedback)

            # Send result
            result = SetGripperPosition.Result()
            result.success = True
            result.message = f"Arm moved to x={x:.3f}, z={z:.3f}"
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error(f"IK error: {e}")
            result = SetGripperPosition.Result()
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

def main(args=None):
    rclpy.init(args=args)
    node = OpenManipulatorIKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
