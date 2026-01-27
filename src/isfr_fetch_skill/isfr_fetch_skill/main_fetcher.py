import rclpy
import math
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion

from .location_check import LocationChecker
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

map_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)



class BottleManager(Node):
    def __init__(self):
        super().__init__('bottle_manager')

        # === NEW: runtime parameter ===
        self.declare_parameter('start', False)

        self.checker = LocationChecker(self)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(
            OccupancyGrid,
            '/knowledge/occupancy',
            self.knowledge_map_callback,
            map_qos
        )

        self.latest_knowledge_map = None
        self.current_goal = None

        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info("Bottle Manager gestart (knowledge-aware).")

    def knowledge_map_callback(self, msg):
        self.latest_knowledge_map = msg

    # ==========================
    # MAIN CONTROL LOOP
    # ==========================
    def control_loop(self):
        if self.current_goal is not None:
            return

        if self.latest_knowledge_map is None:
            self.get_logger().info("⏳ Wachten op knowledge grid...", throttle_duration_sec=3.0)
            return

        start = self.get_parameter('start').value

        if start:
            self.get_logger().info("🧠 Knowledge scoring AAN")
            target = self.select_best_cell_from_grid(self.latest_knowledge_map)
        else:
            self.get_logger().info("📍 Knowledge scoring UIT (fallback)")
            return

        if target is None:
            self.get_logger().warn("❌ Geen geschikt doel gevonden in knowledge grid.")
            return

        tx, ty = target
        approach = self.checker.get_safe_approach_point(tx, ty, stop_dist=0.55)

        if approach:
            gx, gy, yaw = approach
            self.current_goal = (gx, gy)
            self.send_goal(gx, gy, yaw)
        else:
            self.get_logger().warn("❌ Doel onbereikbaar volgens costmap.")

    # ==========================
    # KNOWLEDGE GRID SCORING
    # ==========================
    def select_best_cell_from_grid(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y

        data = np.array(map_msg.data).reshape((height, width))

        BEST_THRESHOLD = 30
        best_score = 0
        best_cell = None

        for y in range(height):
            for x in range(width):
                value = data[y, x]
                if value > BEST_THRESHOLD and value > best_score:
                    best_score = value
                    best_cell = (x, y)

        if best_cell is None:
            return None

        cx, cy = best_cell
        wx = ox + (cx + 0.5) * res
        wy = oy + (cy + 0.5) * res

        self.get_logger().info(f"⭐ Beste knowledge cel: score={best_score} @ ({wx:.2f},{wy:.2f})")
        return wx, wy

    # ==========================
    # FALLBACK (simpel)
    # ==========================
    def select_first_detected_cell(self, map_msg):
        data = np.array(map_msg.data)
        idx = np.argmax(data)
        if data[idx] < 30:
            return None

        w = map_msg.info.width
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y

        y = idx // w
        x = idx % w

        wx = ox + (x + 0.5) * res
        wy = oy + (y + 0.5) * res
        return wx, wy

    # ==========================
    # NAVIGATION
    # ==========================
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)

        self._nav_client.wait_for_server()
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.current_goal = None
            return
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.current_goal = None

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = BottleManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
