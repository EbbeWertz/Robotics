#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
from isfr_bot_msgs.action import FetchObject # Zorg dat deze package gebuild is

from .location_check import LocationChecker

map_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

class BottleFetcherActionServer(Node):
    def __init__(self):
        super().__init__('bottle_fetcher_server')

        self.checker = LocationChecker(self)
        
        # Nav2 Action Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Knowledge Map Subscription
        self.latest_map = None
        self.create_subscription(OccupancyGrid, '/knowledge/occupancy', self.map_callback, map_qos)

        # Onze eigen Action Server
        self._action_server = ActionServer(
            self,
            FetchObject,
            'fetch_object',
            execute_callback=self.execute_callback
        )

        self.get_logger().info("Fetch Action Server is gestart en wacht op goals...")

    def map_callback(self, msg):
        self.latest_map = msg

    async def execute_callback(self, goal_handle):
        """Wordt getriggerd door: ros2 action send_goal /fetch_object ... {label: 'bottle'}"""
        target_label = goal_handle.request.label
        self.get_logger().info(f"Ontvangen opdracht: Zoek naar '{target_label}'")

        if self.latest_map is None:
            self.get_logger().error("Geen knowledge map beschikbaar!")
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Map niet gevonden")

        # 1. Zoek de beste locatie in de grid (Knowledge scoring)
        target = self.select_best_cell(self.latest_map)
        
        if target is None:
            self.get_logger().warn(f"Geen object met label '{target_label}' gevonden in grid.")
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Object niet gevonden in grid")

        tx, ty = target
        
        # 2. Bereken veilige approach pose
        approach = self.checker.get_safe_approach_point(tx, ty, stop_dist=0.55)
        
        if not approach:
            self.get_logger().error("Doel gevonden, maar locatie is onbereikbaar!")
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Locatie onbereikbaar")

        gx, gy, yaw = approach
        
        # 3. Navigeer naar de fles via Nav2
        self.get_logger().info(f"Navigeren naar approach punt: ({gx:.2f}, {gy:.2f})")
        nav_success = await self.go_to_pose(gx, gy, yaw)

        if nav_success:
            self.get_logger().info("Succesvol aangekomen bij de fles!")
            goal_handle.succeed()
            return FetchObject.Result(success=True, message="Aangekomen bij doel")
        else:
            self.get_logger().error("Navigatie naar de fles is mislukt.")
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Navigatie mislukt")

    def select_best_cell(self, map_msg):
        """Vindt de hoogste score in de occupancy grid (bottle heatmap)"""
        width, height = map_msg.info.width, map_msg.info.height
        res = map_msg.info.resolution
        ox, oy = map_msg.info.origin.position.x, map_msg.info.origin.position.y
        
        data = np.array(map_msg.data).reshape((height, width))
        if np.max(data) < 30: # Threshold
            return None

        y, x = np.unravel_index(np.argmax(data), data.shape)
        wx = ox + (x + 0.5) * res
        wy = oy + (y + 0.5) * res
        return wx, wy

    async def go_to_pose(self, x, y, yaw):
        """Stuurt goal naar Nav2 en wacht op resultaat"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        self._nav_client.wait_for_server()
        send_goal_future = await self._nav_client.send_goal_async(goal_msg)
        
        if not send_goal_future.accepted:
            return False

        result_future = await send_goal_future.get_result_async()
        return result_future.status == 4 # STATUS_SUCCEEDED

def main(args=None):
    rclpy.init(args=args)
    node = BottleFetcherActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()