import math

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class GroundTruthOdomPublisher:
    def init(self, webots_node, properties):
        rclpy.init(args=None)

        self.__robot = webots_node.robot          # Supervisor
        self.__node = rclpy.create_node('odom_gt_publisher')

        self.__node.get_logger().info('  - properties: ' + str(properties))
        self.__node.get_logger().info('  - robot name: ' + self.__robot.getName())
        self.__node.get_logger().info('  - is supervisor? ' + str(self.__robot.getSupervisor()))

        # Get robot DEF name from properties or hardcode it
        robot_def = properties.get('robotDef', 'MY_ROBOT')
        self.__robot_node = self.__robot.getFromDef(robot_def)

        if self.__robot_node is None:
            raise RuntimeError(f'No node found with DEF "{robot_def}"')

        self.__odom_pub = self.__node.create_publisher(Odometry, '/odom', 10)
        self.__tf_broadcaster = tf2_ros.TransformBroadcaster(self.__node)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # --- Ground truth pose ---
        pos = self.__robot_node.getPosition()      # [x, y, z]
        ori = self.__robot_node.getOrientation()   # 3x3 rotation matrix (row-major)

        # Yaw from rotation matrix
        # Webots orientation matrix:
        # [ r00 r01 r02
        #   r10 r11 r12
        #   r20 r21 r22 ]
        yaw = math.atan2(ori[2], ori[0])

        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        now = self.__node.get_clock().now().to_msg()

        # --- Odometry message ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[2]
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        vel = self.__robot_node.getVelocity()
        # vel = [vx, vy, vz, wx, wy, wz]
        odom.twist.twist.linear.x = vel[0]
        odom.twist.twist.linear.y = vel[2]
        odom.twist.twist.angular.z = vel[4]

        self.__odom_pub.publish(odom)

        # --- TF ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[2]
        t.transform.translation.z = 0.0

        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.__tf_broadcaster.sendTransform(t)

