#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, Vector3Stamped

import tf2_ros


class GroundTruthOdomPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value

        # State storage
        self.position = None
        self.linear_velocity = None
        self.imu_msg = None

        # Subscribers
        self.create_subscription(
            PointStamped,
            '/TurtleBot3Waffle/gps_sensor',
            self.gps_callback,
            10
        )

        self.create_subscription(
            Vector3Stamped,
            '/TurtleBot3Waffle/gps_sensor/speed_vector',
            self.speed_callback,
            10
        )

        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(0.02, self.publish_odom)  # 50 Hz

        self.get_logger().info('Ground-truth odometry node started.')

    def gps_callback(self, msg):
        self.position = msg.point

    def speed_callback(self, msg):
        self.linear_velocity = msg.vector

    def imu_callback(self, msg):
        self.imu_msg = msg

    def publish_odom(self):
        if self.position is None or self.imu_msg is None:
            return

        now = self.get_clock().now().to_msg()

        # --- Odometry message ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame

        # Position (ground truth from GPS)
        odom.pose.pose.position.x = self.position.x
        odom.pose.pose.position.y = self.position.y
        odom.pose.pose.position.z = 0.0  # planar robot

        # Orientation (ground truth from IMU)
        odom.pose.pose.orientation = self.imu_msg.orientation

        # Linear velocity (ground truth from GPS)
        if self.linear_velocity is not None:
            odom.twist.twist.linear.x = self.linear_velocity.x
            odom.twist.twist.linear.y = self.linear_velocity.y
            odom.twist.twist.linear.z = 0.0

        # Angular velocity (from IMU)
        odom.twist.twist.angular = self.imu_msg.angular_velocity

        # Zero covariance → ground truth
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36

        self.odom_pub.publish(odom)

        # --- TF ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame

        t.transform.translation.x = self.position.x
        t.transform.translation.y = self.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.imu_msg.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
