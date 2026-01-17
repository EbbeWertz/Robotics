from rosgraph_msgs.msg import Clock
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math


class GroundTruthOdomPublisher:
    def init(self, webots_node, properties):
        rclpy.init(args=None)
        self.__robot = webots_node.robot
        self.__node = rclpy.create_node('odom_gt_publisher')
        self.__node.get_logger().info('  - properties: ' + str(properties))
        self.__node.get_logger().info('  - robot name: ' + str(self.__robot.getName()))
        self.__node.get_logger().info('  - basic timestep: ' + str(int(self.__robot.getBasicTimeStep())))
        self.__node.get_logger().info('  - is supervisor? ' + str(self.__robot.getSupervisor()))

        self.__odom_pub = self.__node.create_publisher(Odometry, '/odom-gt', 10)
        self.__tf_broadcaster = tf2_ros.TransformBroadcaster(self.__node)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # read ground truth
        pos = self.__robot.getPosition()
        ori = self.__robot.getOrientation()

        # convert rotation matrix to yaw
        yaw = math.atan2(ori[1], ori[0])

        # publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.__node.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = pos[0]
        odom_msg.pose.pose.position.y = pos[2]  # Webots Z axis -> ROS Y
        odom_msg.pose.pose.position.z = 0
        # convert yaw to quaternion
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        self.__odom_pub.publish(odom_msg)

        # publish TF
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[2]
        t.transform.translation.z = 0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.__tf_broadcaster.sendTransform(t)
